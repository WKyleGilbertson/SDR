#include <cstdio>
#include <cstdlib>
#include <vector>
#include <string>
#include "AcquisitionMgr.hpp"
#include "PCSEngine.hpp"
#include "L1IFUtil.hpp"
#include "ChannelProcessor.h"
#include "g2init.h"

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
        base_acq.codePhase
    };

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

void writeReplayTrackingHeader(FILE * csv) {
fprintf(csv,
    "ms,sample_count,code_phase,doppler,carrier_nco_hz,code_nco_hz,"
    "Ei,Eq,Pi,Pq,Li,Lq,"
    "E,P,L,pll,dll,snr,is_locked\n");};

void writeReplayTrackingRow(FILE * csv, size_t ms, CorrelatorResult r) {
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

/*
writeHandoffHeader(csv);
writeHandoffRow(csv, acq, track_acq, optional_refined_acq);
*/
float pcsToTrackerCodePhase(float pcsCode)
{
    // EPL reg Prompt is bit 32 which is 16 samples or 2 chips
    // FFT size is 16384 not 16386 which is off by 16 or 1 chip
    // Those two together are 3 chips
    float trackerCode = 1023.0f - pcsCode + 3.0f;

    while (trackerCode < 0.0f)
        trackerCode += 1023.0f;

    while (trackerCode >= 1023.0f)
        trackerCode -= 1023.0f;

    return trackerCode;
}

static AcqResult refineHandoffWithTracker(
    const ReplayMeta &meta,
    const std::vector<RawSample> &samples,
    const AcqResult &base_acq)
{
    const size_t ms_samples = meta.fs_rate / 1000;
    const int refine_ms = 20;

    AcqResult best = base_acq;
    float bestP = -1.0f;

    float centers[] = {
        pcsToTrackerCodePhase(base_acq.codePhase)
    };

    for (float center : centers)
    {
        //for (int sampleOffset = -64; sampleOffset <= 64; ++sampleOffset)
        for (int sampleOffset = -1; sampleOffset <= 48; ++sampleOffset)
        {
            AcqResult trial = base_acq;
            trial.codePhase = center + ((float)sampleOffset / 16.0f);

            while (trial.codePhase < 0.0f)
                trial.codePhase += 1023.0f;
            while (trial.codePhase >= 1023.0f)
                trial.codePhase -= 1023.0f;

            G2INIT sv(trial.prn, 0);
            ChannelProcessor chan((double)meta.fs_rate, trial, sv);

            chan.setInputIsComplex(meta.input_is_complex);
            chan.setSampleGain(8.0f);
            chan.setLoopEnables(false, false);

            double Psum = 0.0;

            for (int ms = 0; ms < refine_ms; ++ms)
            {
                CorrelatorResult r =
                    chan.Correlator(samples.data() + ms * ms_samples,
                                    ms_samples);
                Psum += r.P_mag;
            }

            float Pavg = (float)(Psum / refine_ms);

            if (Pavg > bestP)
            {
                bestP = Pavg;
                best = trial;
                best.snr = Pavg;
            }
        }
    }

    printf("[HANDOFF] coarse code=%.4f bin=%d refined code=%.4f metric=%.3f\n",
           base_acq.codePhase,
           base_acq.bin,
           best.codePhase,
           bestP);

    return best;
}



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

if (enableCodePhaseSweep) {
  runCodePhaseSweep(meta, samples, acq);
  return 0;
}

AcqResult track_acq =
    refineHandoffWithTracker(meta, samples, acq);

  printf("[TRK INIT OVERRIDE] acq code=%.4f track code=%.4f\n",
         acq.codePhase,
         track_acq.codePhase);

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