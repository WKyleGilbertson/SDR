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
L1IFStream Card: FNLN Mapping
       =========================================================
       Format: 2-bit Sign-Magnitude, 2 complex samples per byte
       Basackwards Logic: Sample 0 (Older) is in bits 7-4
                          Sample 1 (Newer) is in bits 3-0

       One Byte (8 bits) Layout:
       ---------------------------------------------------------
       | Bit 7 | Bit 6 | Bit 5 | Bit 4 | Bit 3 | Bit 2 | Bit 1 | Bit 0 |
       |-------|-------|-------|-------|-------|-------|-------|-------|
       |  Q0_s |  Q0_m |  I0_s |  I0_m |  Q1_s |  Q1_m |  I1_s |  I1_m |
       ---------------------------------------------------------
       |   Sample 0 (First in Time)    |   Sample 1 (Second in Time)   | */
inline void unpackL1IF(uint8_t b, kiss_fft_cpx& c0, kiss_fft_cpx& c1) {
    auto map = [](uint8_t m, uint8_t s) -> int16_t {
        // Mapping: Sign 0 -> Pos, Sign 1 -> Neg
        // Mapping: Mag 0 -> 1, Mag 1 -> 3
        int16_t val = (s == 0) ? 1 : -1;
        if (m != 0) val *= 3;
        return val << 3; // Shifted for fixed-point headroom
    };

    // SAMPLE 0: The "First Nibble" (High bits 7-4)
    // I0: Mag=Bit 4, Sign=Bit 5 | Q0: Mag=Bit 6, Sign=Bit 7
    c0.r = map((b >> 4) & 1, (b >> 5) & 1); 
    c0.i = map((b >> 6) & 1, (b >> 7) & 1); 
    
    // SAMPLE 1: The "Low Nibble" (Low bits 3-0)
    // I1: Mag=Bit 0, Sign=Bit 1 | Q1: Mag=Bit 2, Sign=Bit 3
    c1.r = map((b >> 0) & 1, (b >> 1) & 1); 
    c1.i = map((b >> 2) & 1, (b >> 3) & 1); 
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