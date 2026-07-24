#include "HandoffRefiner.hpp"

#include <cmath>
#include <cstdio>
#include "ChannelProcessor.h"
#include "g2init.h"

float wrapCodePhase(float code)
{
  while (code < 0.0f)
    code += 1023.0f;
  while (code >= 1023.0f)
    code -= 1023.0f;
  return code;
}

static HandoffMetric evaluateHandoffCandidate(
    double fs_rate,
    bool input_is_complex,
    const RawSample *samples,
    const AcqResult &trial,
    int refine_ms)
{
  const size_t ms_samples = (size_t)(fs_rate / 1000.0);

  G2INIT sv(trial.prn, 0);
  ChannelProcessor chan(fs_rate, trial, sv, false);

  chan.setInputIsComplex(input_is_complex);
  chan.setSampleGain(8.0f);
  chan.setLoopEnables(false, false);

  double Esum = 0.0;
  double Psum = 0.0;
  double Lsum = 0.0;
  double dllSum = 0.0;
  double pllSum = 0.0;
  int64_t PiSum = 0;
  int64_t PqSum = 0;

  for (int ms = 0; ms < refine_ms; ++ms)
  {
    CorrelatorResult r =
        chan.Correlator(samples + ms * ms_samples, ms_samples);

    Esum += r.E_mag;
    Psum += r.P_mag;
    Lsum += r.L_mag;
    dllSum += r.code_error;
    pllSum += r.carrier_phase_error;
    PiSum += r.Pi;
    PqSum += r.Pq;
  }

  HandoffMetric m;
  m.E = (float)(Esum / refine_ms);
  m.P = (float)(Psum / refine_ms);
  m.L = (float)(Lsum / refine_ms);
  m.dll = (float)(dllSum / refine_ms);
  m.pll = (float)(pllSum / refine_ms);
  m.Pi = PiSum / refine_ms;
  m.Pq = PqSum / refine_ms;
  return m;
}

AcqResult refineHandoffWithTracker(
    double fs_rate,
    bool input_is_complex,
    const RawSample *samples,
    size_t sample_count,
    const AcqResult &base_acq)
{
  const int refine_ms = 5;
  const size_t ms_samples = (size_t)(fs_rate / 1000.0);

  if (!samples || sample_count < (size_t)refine_ms * ms_samples)
    return base_acq;

  AcqResult best = base_acq;
  float bestP = -1.0f;

  FILE *csv = fopen("handoff_refine.csv", "w");
  if (csv)
  {
    fprintf(csv,
            "prn,input_code,input_bin,"
            "stage,offset,chip_offset,test_code,"
            "E,P,L,dll,pll,Pi,Pq,is_best\n");
  }

  auto testCandidate =
      [&](const char *stage, int offset, float chipOffset, float codePhase)
  {
    AcqResult trial = base_acq;
    trial.codePhase = wrapCodePhase(codePhase);

    HandoffMetric m =
        evaluateHandoffCandidate(
            fs_rate,
            input_is_complex,
            samples,
            trial,
            refine_ms);

    if (csv)
    {
      fprintf(csv,
              "%d,%.6f,%d,%s,%d,%.6f,%.6f,"
              "%.6f,%.6f,%.6f,%.9f,%.9f,%lld,%lld,%d\n",
              base_acq.prn,
              base_acq.codePhase,
              base_acq.bin,
              stage,
              offset,
              chipOffset,
              trial.codePhase,
              m.E,
              m.P,
              m.L,
              m.dll,
              m.pll,
              (long long)m.Pi,
              (long long)m.Pq,
              0);
    }

    if (m.P > bestP)
    {
      bestP = m.P;
      best = trial;
      best.snr = m.P;
    }
  };

  for (int chipOffset = -3; chipOffset <= 3; ++chipOffset)
  {
    testCandidate(
        "chip",
        chipOffset,
        (float)chipOffset,
        base_acq.codePhase + (float)chipOffset);
  }

  AcqResult stage1_best = best;

  for (int sampleOffset = -4; sampleOffset <= 4; ++sampleOffset)
  {
    float chipOffset = (float)sampleOffset / 16.0f;

    testCandidate(
        "sample",
        sampleOffset,
        chipOffset,
        stage1_best.codePhase + chipOffset);
  }

  printf("[HANDOFF] input code=%.4f bin=%d refined code=%.4f metric=%.3f\n",
         base_acq.codePhase,
         base_acq.bin,
         best.codePhase,
         bestP);

  if (csv)
    fclose(csv);

  return best;
}

float pcsToTrackerCodePhase(float pcsCode)
{
  float trackerCode = 1023.0f - pcsCode + 3.0f;
  return wrapCodePhase(trackerCode);
}