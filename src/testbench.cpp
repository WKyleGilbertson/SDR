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

//    FILE *csv = fopen("code_phase_sweep.csv", "w");
//    if (!csv)
//        return;

    /*
    fprintf(csv,
            "sample_offset,chip_offset,code_phase,E,P,L,dll,Pi,Pq,pll,snr\n");
            */

    for (int sampleOffset = -96;
         sampleOffset <= -64;
         ++sampleOffset)
    {
        float chipOffset =
            (float)sampleOffset / 16.0f;

        AcqResult acq = base_acq;

        acq.codePhase =
            base_acq.codePhase + chipOffset;

        while (acq.codePhase < 0.0f)
            acq.codePhase += 1023.0f;

        while (acq.codePhase >= 1023.0f)
            acq.codePhase -= 1023.0f;

        G2INIT sv(acq.prn, 0);

        ChannelProcessor chan(
            (double)meta.fs_rate,
            acq,
            sv);

        chan.setInputIsComplex(
            meta.input_is_complex);

        CorrelatorResult r =
            chan.Correlator(
                samples.data(),
                ms_samples);

        /*
        fprintf(csv,
                "%d,%.6f,%.6f,%.6f,%.6f,%.6f,%.9f,%d,%d,%.9f,%.3f\n",
                sampleOffset,
                chipOffset,
                acq.codePhase,
                r.E_mag,
                r.P_mag,
                r.L_mag,
                r.code_error,
                r.Pi,
                r.Pq,
                r.carrier_phase_error,
                r.snr); */
    }

//    fclose(csv);

//    printf("[OK] wrote code_phase_sweep.csv\n");
}

int main(int argc, char **argv)
{
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

//  runCodePhaseSweep(meta, samples, acq);

  AcqResult track_acq = acq;
  track_acq.codePhase = 170.375f;

  printf("[TRK INIT OVERRIDE] acq code=%.4f track code=%.4f\n",
         acq.codePhase,
         track_acq.codePhase);

  G2INIT sv(track_acq.prn, 0);

  ChannelProcessor chan(
      (double)meta.fs_rate,
      track_acq,
      sv);

  chan.setInputIsComplex(meta.input_is_complex);

  FILE *csv = fopen("replay_tracking.csv", "w");
fprintf(csv,
    "ms,sample_count,code_phase,doppler,"
    "Ei,Eq,Pi,Pq,Li,Lq,"
    "E,P,L,pll,dll,snr,is_locked\n");

  for (size_t ms = 0; ms < ms_count; ++ms)
  {
    CorrelatorResult r =
        chan.Correlator(samples.data() + ms * ms_samples, ms_samples);

fprintf(csv,
    "%zu,%zu,%.6f,%.3f,"
    "%d,%d,%d,%d,%d,%d,"
    "%.3f,%.3f,%.3f,%.9f,%.9f,%.3f,%d\n",
    ms,
    r.epoch_sample_count,
    r.code_phase,
    r.doppler_hz,
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
  }

  fclose(csv);

  printf("[OK] wrote replay_tracking.csv using %zu ms\n", ms_count);
  return 0;
}