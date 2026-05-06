#ifndef PCSEngine_HPP
#define PCSEngine_HPP

#include <vector>
#include <complex>
#include <map>
#include <cstdint>
#include "G2INIT.h"
#include "NCO.h" // Must include this for m_nco member

#undef kiss_fft_scalar
#define kiss_fft_scalar int16_t
#define FIXED_POINT 16
#include "kiss_fft.h"

struct AcqResult
{
    int prn;                // Satellite PRN number
    int bin;                // Doppler frequency bin index
    int peakIndex;          // Sample index of the correlation peak
    double peakMagnitude;   // Absolute power of the correlation peak
    double snr;             // Signal-to-Noise Ratio in dB
    float phase;            // Carrier Phase at the peak (in radians)
    uint32_t sampleTick;     // Phase of incoming sample stream
    float codePhase;         // sub-ms code phase (0.0 - 1022.99)
};

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
    std::map<int, G2INIT> m_sv_list;

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
    G2INIT getSV(int prn) { return m_sv_list.at(prn); }
    AcqResult search(int prn, const std::vector<kiss_fft_cpx> &rawData,
                     float centerFreq, int binRange, float binWidth,
                    uint32_t sampleTick = 0);
};

#endif