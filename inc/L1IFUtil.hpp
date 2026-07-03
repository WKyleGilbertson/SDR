// L1IFUtil.hpp - Utility functions for L1 Interface
#pragma once
#include <iostream>
#include <ctime>
#include <iomanip>
#include <sstream>
#include <string>
#include <cstdint>
#include <cstdio>
namespace ReceiverConfig
{
    constexpr int    CA_CHIPS          = 1023;
    constexpr int    SAMPLES_PER_CHIP  = 16;
    constexpr int    SAMPLES_PER_MS    = CA_CHIPS * SAMPLES_PER_CHIP; // 16368
    constexpr int    PCS_FFT_SIZE      = 16384;

    constexpr float  DEF_SAMPLE_RATE   = 16368000.0f;

    constexpr double CODE_FREQ_HZ      = 1.023e6;
    constexpr double L1_IF_HZ          = 4.092e6;

    constexpr int    EPL_PROMPT_BIT    = 32;
    constexpr int    EPL_EARLY_BIT     = 24;  // example only
    constexpr int    EPL_LATE_BIT      = 40;  // example only

    constexpr int    CHIP_TRAVEL_DELAY = 0;   // once finalized
}

#pragma pack(push, 1)
struct RFE_Header_t
{
    uint8_t pkt_type;
    uint32_t fs_rate;
    uint32_t unix_time;
    uint32_t sample_tick;
    uint32_t seq_num;
    char dev_tag[16];
    uint16_t payload_len;
};
#pragma pack(pop)

std::string get_iso8601_timestamp(uint32_t unix_time, uint16_t ms_offset);
struct TimeTrio {
    uint32_t unixSecond;
    uint16_t msCount;
    uint8_t  fracMS;
};

TimeTrio get_timeData(uint32_t unixSeconds, uint32_t sampleTick, uint32_t Fs);


/**
 * L1IFStream Bit Unpacking (MAX2769 bit-packed format)
 * =========================================================
 * Format: 2-bit Sign-Magnitude, 2 complex samples per byte.
 * * FNHN (Default Hardware Mapping): 
 * Sample 0 (Older) is in the High Nibble [7:4]
 * Sample 1 (Newer) is in the Low Nibble  [3:0]
 *
 * Byte Layout (8 bits):
 * ---------------------------------------------------------
 * | Bit 7 | Bit 6 | Bit 5 | Bit 4 | Bit 3 | Bit 2 | Bit 1 | Bit 0 |
 * |-------|-------|-------|-------|-------|-------|-------|-------|
 * |  Q0_s |  Q0_m |  I0_s |  I0_m |  Q1_s |  Q1_m |  I1_s |  I1_m |
 * ---------------------------------------------------------
 * |   Sample 0 (Older / High)     |   Sample 1 (Newer / Low)      |
 * * Mapping Logic:
 * Sign (s): 0 -> Positive, 1 -> Negative
 * Mag  (m): 0 -> 1 (Low),  1 -> 3 (High)
 */
struct RawSample
{
    int8_t i;             // Raw unscaled I value directly from MAX2769
    int8_t q;             // Raw unscaled Q value directly from MAX2769
    uint32_t sample_tick; // Inherited directly from meta.sample_tick
    uint32_t unix_time;
    uint64_t sample_index;
};

struct ComplexSample {
    int16_t i;
    int16_t q;
};
struct UnpackEntry {
    ComplexSample s0;
    ComplexSample s1;
};

// Accessor functions instead of direct array access
const UnpackEntry* GetLUT_FNHN();
const UnpackEntry* GetLUT_FNLN();

inline void unpackL1IF_LUT(uint8_t b, const UnpackEntry* lut, ComplexSample& out0, ComplexSample& out1) {
    const UnpackEntry& entry = lut[b];
    out0 = entry.s0;
    out1 = entry.s1;
}

struct CorrelatorResult;

void printCorrelatorData(FILE * fp, CorrelatorResult &res);