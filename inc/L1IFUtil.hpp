// L1IFUtil.hpp - Utility functions for L1 Interface
#pragma once
#include <iostream>
#include <ctime>
#include <iomanip>
#include <sstream>
#include <string>
#include <cstdint>

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