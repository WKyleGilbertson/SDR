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
    int bin;
    int peakIndex;
    double peakMagnitude;
    double snr;
};
/**
 * unpackL1IF: MAX2769 Split-Nibble Unpacker
 * Based on the established Constitutional Law of raw2bin.cpp
 * 1 Byte = 2 Complex Samples (I0, Q0, I1, Q1)
 */
inline void unpackL1IF(uint8_t b, kiss_fft_cpx& c0, kiss_fft_cpx& c1) {
    // MAX2769 Sign-Magnitude mapping:
    // Sign 0 -> Positive, Sign 1 -> Negative
    // Mag  0 -> 1, Mag 1 -> 3
    auto map = [](uint8_t m, uint8_t s) -> int16_t {
        int16_t val = (s == 0) ? 1 : -1;
        if (m != 0) val *= 3;
        return val << 3; // Shifted for fixed-point FFT headroom
    };

    // Low Nibble: Magnitudes (0-3) | High Nibble: Signs (4-7)
    c0.r = map((b >> 0) & 1, (b >> 4) & 1); // I0
    c0.i = map((b >> 1) & 1, (b >> 5) & 1); // Q0
    
    c1.r = map((b >> 2) & 1, (b >> 6) & 1); // I1
    c1.i = map((b >> 3) & 1, (b >> 7) & 1); // Q1
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
    std::vector<kiss_fft_cpx> m_ncoBuffer; // Initialize to size N (16384)

    kiss_fft_cfg m_cfg_fwd;
    kiss_fft_cfg m_cfg_inv;

    std::map<int, std::vector<kiss_fft_cpx>> codeFfts;


    static inline void complex_mix(kiss_fft_cpx* out, const kiss_fft_cpx* a, 
                                   const kiss_fft_cpx* b, size_t count, size_t shift) 
    {
        for (size_t i = 0; i < count; ++i) {
            int32_t r = ((int32_t)a[i].r * b[i].r - (int32_t)a[i].i * b[i].i);
            int32_t im = ((int32_t)a[i].r * b[i].i + (int32_t)a[i].i * b[i].r);
            out[i].r = (int16_t)(r>>shift);
            out[i].i = (int16_t)(im>>shift);
        }
    }

public:
    PCSEngine(double sampleFreq);
    ~PCSEngine();
    void initPrn(int prn);
    AcqResult search(int prn, const std::vector<kiss_fft_cpx> &rawData,
                     float centerFreq, int binRange, float binWidth);
};

#endif