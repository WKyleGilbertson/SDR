#include <cstdio>
#include <cstdlib>
#include <vector>
#include <string>
#include <cmath>
#include "AcquisitionMgr.hpp"
#include "PCSEngine.hpp"
#include "L1IFUtil.hpp"
#include "ChannelProcessor.h"
#include "g2init.h"
#include "HandoffRefiner.hpp"

struct ReplayMeta
{
  uint32_t fs_rate = 16368000;
  int prn = 0;
  int bin = 0;
  float codePhase = 0.0f;
  float snr = 0.0f;
  bool input_is_complex = false;
  size_t sample_count = 0;
};

static bool readMeta(const char *path, ReplayMeta &m)
{
  FILE *fp = fopen(path, "r");
  if (!fp)
    return false;

  char key[64];
  char val[128];

  while (fscanf(fp, "%63[^=]=%127s\n", key, val) == 2)
  {
    std::string k(key);
    if (k == "fs_rate")
      m.fs_rate = (uint32_t)strtoul(val, nullptr, 10);
    else if (k == "prn")
      m.prn = atoi(val);
    else if (k == "bin")
      m.bin = atoi(val);
    else if (k == "codePhase")
      m.codePhase = (float)atof(val);
    else if (k == "snr")
      m.snr = (float)atof(val);
    else if (k == "input_is_complex")
      m.input_is_complex = atoi(val) != 0;
    else if (k == "sample_count")
      m.sample_count = (size_t)strtoull(val, nullptr, 10);
  }

  fclose(fp);
  return true;
}

static bool loadSamples(const char *path, std::vector<RawSample> &samples)
{
  FILE *fp = fopen(path, "rb");
  if (!fp)
    return false;

  fseek(fp, 0, SEEK_END);
  long bytes = ftell(fp);
  rewind(fp);

  samples.resize(bytes / sizeof(RawSample));
  fread(samples.data(), sizeof(RawSample), samples.size(), fp);
  fclose(fp);
  return true;
}

static void runCodePhaseSweep(
    const ReplayMeta &meta,
    const std::vector<RawSample> &samples,
    const AcqResult &base_acq)
{
  const size_t ms_samples = meta.fs_rate / 1000;

  FILE *csv = fopen("code_phase_sweep.csv", "w");
  if (!csv)
    return;

  fprintf(csv,
          "candidate,chip_offset,code_phase,E,P,L,dll,Pi,Pq,pll\n");

  float centers[] = {
      45.3125f,
      170.3750f,
      980.6250f,
      base_acq.codePhase};

  const int sweep_ms = 20;

  for (float center : centers)
  {
    for (int sampleOffset = -64; sampleOffset <= 64; ++sampleOffset)
    {
      float chipOffset = (float)sampleOffset / 16.0f;

      AcqResult acq = base_acq;
      acq.codePhase = center + chipOffset;

      while (acq.codePhase < 0.0f)
        acq.codePhase += 1023.0f;

      while (acq.codePhase >= 1023.0f)
        acq.codePhase -= 1023.0f;

      G2INIT sv(acq.prn, 0);

      ChannelProcessor chan(
          (double)meta.fs_rate,
          acq,
          sv);

      chan.setInputIsComplex(meta.input_is_complex);
      chan.setSampleGain(8.0f);
      chan.setLoopEnables(false, false);

      double E_sum = 0.0;
      double P_sum = 0.0;
      double L_sum = 0.0;
      double dll_sum = 0.0;
      double pll_sum = 0.0;
      int64_t Pi_sum = 0;
      int64_t Pq_sum = 0;

      for (int ms = 0; ms < sweep_ms; ++ms)
      {
        CorrelatorResult r =
            chan.Correlator(
                samples.data() + ms * ms_samples,
                ms_samples);

        E_sum += r.E_mag;
        P_sum += r.P_mag;
        L_sum += r.L_mag;
        dll_sum += r.code_error;
        pll_sum += r.carrier_phase_error;
        Pi_sum += r.Pi;
        Pq_sum += r.Pq;
      }

      fprintf(csv,
              "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.9f,%lld,%lld,%.9f\n",
              center,
              chipOffset,
              acq.codePhase,
              E_sum / sweep_ms,
              P_sum / sweep_ms,
              L_sum / sweep_ms,
              dll_sum / sweep_ms,
              (long long)(Pi_sum / sweep_ms),
              (long long)(Pq_sum / sweep_ms),
              pll_sum / sweep_ms);
    }
  }

  fclose(csv);

  printf("[OK] wrote code_phase_sweep.csv\n");
}

void writeReplayTrackingHeader(FILE *csv)
{
  fprintf(csv,
          "ms,sample_count,code_phase,doppler,carrier_nco_hz,code_nco_hz,"
          "Ei,Eq,Pi,Pq,Li,Lq,"
          "E,P,L,pll,dll,snr,is_locked\n");
};

void writeReplayTrackingRow(FILE *csv, size_t ms, CorrelatorResult r)
{
  fprintf(csv,
          "%zu,%zu,%.6f,%.3f,%.3f,%.3f,"
          "%d,%d,%d,%d,%d,%d,"
          "%.3f,%.3f,%.3f,%.9f,%.9f,%.3f,%d\n",
          ms,
          r.epoch_sample_count,
          r.code_phase,
          r.doppler_hz,
          r.carrier_nco_hz,
          r.code_nco_hz,
          r.Ei,
          r.Eq,
          r.Pi,
          r.Pq,
          r.Li,
          r.Lq,
          r.E_mag,
          r.P_mag,
          r.L_mag,
          r.carrier_phase_error,
          r.code_error,
          r.snr,
          r.is_locked ? 1 : 0);
};

int main(int argc, char **argv)
{
  bool enableSampleTrace = false;
  bool enableReplayTracking = true;
  bool enableHandoffLog = true;
  bool enableCodePhaseSweep = false;

  const char *base = (argc > 1) ? argv[1] : "tracking_capture";

  char metaPath[256];
  char rawPath[256];
  snprintf(metaPath, sizeof(metaPath), "%s.meta", base);
  snprintf(rawPath, sizeof(rawPath), "%s.rawsamples", base);

  ReplayMeta meta;
  if (!readMeta(metaPath, meta))
  {
    printf("[ERR] failed to read %s\n", metaPath);
    return 1;
  }

  std::vector<RawSample> samples;
  if (!loadSamples(rawPath, samples))
  {
    printf("[ERR] failed to read %s\n", rawPath);
    return 1;
  }

  size_t ms_samples = meta.fs_rate / 1000;
  size_t ms_count = samples.size() / ms_samples;

  RFE_Header_t rfe_meta = {};
  rfe_meta.fs_rate = meta.fs_rate;

  PCSEngine pcs((float)meta.fs_rate);
  AcquisitionMgr acqMgr(pcs);

  AcqResult acq =
      acqMgr.runSingle(
          rfe_meta,
          samples.data(),
          5 * ms_samples,
          meta.prn);

  printf("[ACQ] saved  code=%.4f bin=%d snr=%.1f\n",
         meta.codePhase, meta.bin, meta.snr);

  printf("[ACQ] replay code=%.4f bin=%d snr=%.1f\n",
         acq.codePhase, acq.bin, acq.snr);

  const float IF_HZ = 4.092e6f;

  float coarseDopplerHz = acq.bin * 500.0f;

  AcqResult acqFine = acq;
  float fineAbsDopplerHz = coarseDopplerHz;
  /* We don't need a find bin PCS. Serial search gets our answer
  AcqResult acqFine =
      acqMgr.runSingle(
          rfe_meta,
          samples.data(),
          5 * ms_samples,
          meta.prn,
          IF_HZ + coarseDopplerHz,
          2,
          50.0f);

  float fineAbsDopplerHz =
      coarseDopplerHz + acqFine.bin * 50.0f; */

  printf("[ACQ FINE] local_bin=%d abs_dopp=%.1f code=%.4f peak=%d snr=%.1f\n",
         acqFine.bin,
         fineAbsDopplerHz,
         acqFine.codePhase,
         acqFine.peakIndex,
         acqFine.snr);

    float coarseMapped = pcsToTrackerCodePhase(acq.codePhase);
  float fineMapped = pcsToTrackerCodePhase(acqFine.codePhase);

  printf("[MAP] coarse_pcs=%.4f coarse_track=%.4f fine_pcs=%.4f fine_track=%.4f delta_track=%.4f chips\n",
         acq.codePhase,
         coarseMapped,
         acqFine.codePhase,
         fineMapped,
         fineMapped - coarseMapped);

  AcqResult track_input = acq;

  track_input.codePhase = fineMapped;
  track_input.bin = (int)roundf(fineAbsDopplerHz / 500.0f);

  printf("[TRK INPUT] code=%.4f bin=%d fine_abs_dopp=%.1f\n",
         track_input.codePhase,
         track_input.bin,
         fineAbsDopplerHz);

  if (enableCodePhaseSweep)
  {
    runCodePhaseSweep(meta, samples, acq);
    return 0;
  }

  AcqResult track_acq =
      refineHandoffWithTracker(
          (double)meta.fs_rate,
          meta.input_is_complex,
          samples.data(),
          samples.size(),
          track_input);

  printf("[TRK INIT OVERRIDE] acq code=%.4f track code=%.4f\n",
         acq.codePhase,
         track_acq.codePhase);

float pcsA = acqFine.codePhase;

float pcsB16368 =
    (float)((16368 - acqFine.peakIndex) % 16368) / 16.0f;

float pcsB16384 =
    (float)((16384 - acqFine.peakIndex) % 16384) / 16.0f;

float mapped =
    pcsToTrackerCodePhase(pcsA);

printf("[SUMMARY] PRN=%d pcs=%.4f mapped=%.4f refined=%.4f dMapped=%.4f\n",
       acqFine.prn,
       acqFine.codePhase,
       mapped,
       track_acq.codePhase,
       track_acq.codePhase - mapped);

    FILE *cmp = fopen("pcs_vs_tracker_handoff.csv", "w");
if (cmp)
{
    fprintf(cmp,
        "prn,peakIndex,pcsA,pcsB16368,pcsB16384,"
        "mapped,refined,dB16368,dB16384,dMapped\n");

    fprintf(cmp,
        "%d,%d,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n",
        acqFine.prn,
        acqFine.peakIndex,
        pcsA,
        pcsB16368,
        pcsB16384,
        mapped,
        track_acq.codePhase,
        track_acq.codePhase - pcsB16368,
        track_acq.codePhase - pcsB16384,
        track_acq.codePhase - mapped);

    fclose(cmp);
}

  G2INIT sv(track_acq.prn, 0);

  ChannelProcessor chan(
      (double)meta.fs_rate,
      track_acq,
      sv);

  chan.setInputIsComplex(meta.input_is_complex);
  chan.setSampleGain(8.0f);

  if (enableSampleTrace)
    chan.enableSampleTrace("sample_trace.csv", 512);

  FILE *csv = fopen("replay_tracking.csv", "w");
  writeReplayTrackingHeader(csv);

  for (size_t ms = 0; ms < ms_count; ++ms)
  {
    CorrelatorResult r =
        chan.Correlator(samples.data() + ms * ms_samples, ms_samples);

    writeReplayTrackingRow(csv, ms, r);
  }

  fclose(csv);

  printf("[OK] wrote replay_tracking.csv using %zu ms\n", ms_count);
  return 0;
}