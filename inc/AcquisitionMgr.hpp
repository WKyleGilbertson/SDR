#pragma once
#include <vector>
#include <cstdint>
#include <algorithm>
#include "L1IFUtil.hpp"  // Provides RFE_Header_t
#include "PCSEngine.hpp" // Contains AcqResult and unpackL1IF

static constexpr int ACQ_SAMPLE_SHIFT = 3;
static constexpr float ACQ_SNR_THRESHOLD_DB = 9.0f;
class AcquisitionMgr
{
private:
    PCSEngine &m_pcs;
    const size_t FFT_SIZE = 16384;
    const int NUM_MS = 5;

public:
    AcquisitionMgr(PCSEngine &engine) : m_pcs(engine) {}

    std::vector<AcqResult> run(
        const RFE_Header_t &meta,
        RawSample *samples,
        size_t sample_count);
    AcqResult runSingle(
        const RFE_Header_t &meta,
        RawSample *samples,
        size_t sample_count,
        int prn);
    AcqResult runSingle(
        const RFE_Header_t &meta,
        RawSample *samples,
        size_t sample_count,
        int prn,
        float centerFreq,
        int binRange,
        float binWidth);
};
