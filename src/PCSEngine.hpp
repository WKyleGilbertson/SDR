#ifndef PCSEngine_HPP
#define PCSEngine_HPP

#include <vector>
#include <complex>
#include <map>
#include "NCO.h" // Must include this for m_nco member

#undef kiss_fft_scalar
#define kiss_fft_scalar float
#include "kiss_fft.h"

struct AcqResult
{
    int bin;
    int peakIndex;
    double peakMagnitude;
    double snr;
};

class PCSEngine
{
private:
    double m_sampleFreq;
    NCO m_nco;
    int N = 16384;

    std::vector<kiss_fft_cpx> m_workspace;
    std::vector<float> m_accumulatedMag;
    std::vector<kiss_fft_cpx> m_codeFftCurrent; // <--- Add this line

    kiss_fft_cfg m_cfg_fwd;
    kiss_fft_cfg m_cfg_inv;

    std::map<int, std::vector<kiss_fft_cpx>> codeFfts;

public:
    PCSEngine(double sampleFreq);
    ~PCSEngine();
    void initPrn(int prn);
    AcqResult search(int prn, const std::vector<kiss_fft_cpx> &rawData,
                     float centerFreq, int binRange, float binWidth);
};

#endif