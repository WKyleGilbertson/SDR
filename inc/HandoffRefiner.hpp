#pragma once

#include <cstddef>
#include <cstdint>
#include "L1IFUtil.hpp"
#include "PCSEngine.hpp"

struct HandoffMetric
{
  float E = 0.0f;
  float P = 0.0f;
  float L = 0.0f;
  float dll = 0.0f;
  float pll = 0.0f;
  int64_t Pi = 0;
  int64_t Pq = 0;
};

float wrapCodePhase(float code);

float pcsToTrackerCodePhase(float pcsCode);

AcqResult refineHandoffWithTracker(
    double fs_rate,
    bool input_is_complex,
    const RawSample *samples,
    size_t sample_count,
    const AcqResult &base_acq);