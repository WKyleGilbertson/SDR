#ifndef PCSEngine_HPP
#define PCSEngine_HPP

#include <vector>
#include <complex>
#include <map>
#include <cstdint>
#include "NCO.h" // Must include this for m_nco member

#undef kiss_fft_scalar
#define kiss_fft_scalar int16_t
#define FIXED_POINT 16
#include "kiss_fft.h"

struct AcqResult
{
    int bin;                // Doppler frequency bin index
    int peakIndex;          // Sample index of the correlation peak
    double peakMagnitude;   // Absolute power of the correlation peak
    double snr;             // Signal-to-Noise Ratio in dB
    float phase;            // Carrier Phase at the peak (in radians)
    uint32_t sampleTick;     // Phase of incoming sample stream
};

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
inline void unpackL1IF(uint8_t b, kiss_fft_cpx& c0, kiss_fft_cpx& c1, bool isFNLN = false) {
    // Helper to map bits to 16-bit fixed-point values (Shifted for headroom)
    auto map = [](uint8_t m, uint8_t s) -> int16_t {
        int16_t val = (s == 0) ? 1 : -1;
        if (m != 0) val *= 3;
        return val << 3; 
    };

    if (isFNLN) {
        // FNLN: Low nibble contains the earlier sample (Sample 0)
        c0.r = map((b >> 0) & 1, (b >> 1) & 1); // I0
        c0.i = map((b >> 2) & 1, (b >> 3) & 1); // Q0
        c1.r = map((b >> 4) & 1, (b >> 5) & 1); // I1
        c1.i = map((b >> 6) & 1, (b >> 7) & 1); // Q1
    } else {
        // FNHN: High nibble contains the earlier sample (Sample 0)
        c0.r = map((b >> 4) & 1, (b >> 5) & 1); // I0
        c0.i = map((b >> 6) & 1, (b >> 7) & 1); // Q0
        c1.r = map((b >> 0) & 1, (b >> 1) & 1); // I1
        c1.i = map((b >> 2) & 1, (b >> 3) & 1); // Q1
    }
}
class PCSEngine
{
private:
    double m_sampleFreq;
    NCO m_nco;
    int N = 16384; // 2^14 = 16384, 16 more than 16386... zero padding for FFT

    std::vector<float> m_accumulatedMag;
    std::vector<kiss_fft_cpx> m_workspace;
    std::vector<kiss_fft_cpx> m_codeFftCurrent; // <--- Add this line
    std::vector<kiss_fft_cpx> m_ncoBuffer;      // Initialize to size N (16384)

    kiss_fft_cfg m_cfg_fwd;
    kiss_fft_cfg m_cfg_inv;

    kiss_fft_cpx m_lastPeakComplex;

    std::map<int, std::vector<kiss_fft_cpx>> codeFfts;

    static inline void complex_mix(kiss_fft_cpx *out, const kiss_fft_cpx *a,
                                   const kiss_fft_cpx *b, size_t count, size_t shift)
    {
        for (size_t i = 0; i < count; ++i)
        {
            int32_t r = ((int32_t)a[i].r * b[i].r - (int32_t)a[i].i * b[i].i);
            int32_t im = ((int32_t)a[i].r * b[i].i + (int32_t)a[i].i * b[i].r);
            out[i].r = (int16_t)(r >> shift);
            out[i].i = (int16_t)(im >> shift);
        }
    }

public:
    PCSEngine(double sampleFreq);
    ~PCSEngine();
    void initPrn(int prn);
    AcqResult search(int prn, const std::vector<kiss_fft_cpx> &rawData,
                     float centerFreq, int binRange, float binWidth,
                    uint32_t sampleTick = 0);
};

#endif