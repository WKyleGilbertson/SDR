#include <cmath>
#include <algorithm>
#include <iostream>
#include "PCSEngine.hpp"
#include "G2INIT.h"

extern "C"
{
    void KISS_FFT_ERROR(const char *msg)
    {
        std::cerr << "KissFFT Error: " << msg << std::endl;
    }
}

PCSEngine::PCSEngine(double sampleFreq)
    : m_sampleFreq(sampleFreq),
      m_nco(10, (float)sampleFreq),
      m_workspace(N),
      m_accumulatedMag(N),
      m_codeFftCurrent(N),
      m_ncoBuffer(N) // Initialize m_ncoBuffer to size N (16384)
{
    m_cfg_fwd = kiss_fft_alloc(16384, 0, NULL, NULL);
    m_cfg_inv = kiss_fft_alloc(16384, 1, NULL, NULL);
}

PCSEngine::~PCSEngine()
{
    free(m_cfg_fwd);
    free(m_cfg_inv);
}

void PCSEngine::initPrn(int prn)
{
    if (codeFfts.find(prn) != codeFfts.end())
        return;

    G2INIT sv(prn, 0);

    // 1. Fill 1ms exactly (16368 samples)
    for (size_t idx = 0; idx < 16368; idx++)
    {
        m_codeFftCurrent[idx].r = (int16_t)sv.CODE[idx / 16] * 1024;
        m_codeFftCurrent[idx].i = 0;
    }

    // 2. ZERO-PAD the remaining 16 samples to reach N=16384
    // This prevents the "Franken-chip" interference
    for (size_t idx = 16368; idx < N; idx++)
    {
        m_codeFftCurrent[idx].r = 0;
        m_codeFftCurrent[idx].i = 0;
    }

    // 3. Perform Forward FFT
    kiss_fft(m_cfg_fwd, m_codeFftCurrent.data(), m_codeFftCurrent.data());

    // 4. Conjugate (flipped Imaginary)
    for (size_t idx = 0; idx < N; idx++)
    {
        // m_codeFftCurrent[idx].i *= -1.0f;
        m_codeFftCurrent[idx].i = -m_codeFftCurrent[idx].i;
    }

    // 5. Store the clean, padded, frequency-domain replica
    codeFfts[prn] = m_codeFftCurrent;
}

AcqResult PCSEngine::search(int prn, const std::vector<kiss_fft_cpx> &rawData,
                            float centerFreq, int binRange, float binWidth,
                            uint32_t sampleTick)
{
    AcqResult bestResult = {0, 0, -99.0f, -99.0f, 0.0f, 0};
    int numBlocks = (int)(rawData.size() / N);
    if (numBlocks == 0)
        return bestResult;

    initPrn(prn);
    const std::vector<kiss_fft_cpx> &currentCodeFft = codeFfts[prn];

    for (int bin = -binRange; bin <= binRange; bin++)
    {
        std::fill(m_accumulatedMag.begin(), m_accumulatedMag.end(), 0.0f);

        m_nco.SetFrequency(centerFreq + (bin * binWidth));

        for (size_t idx = 0; idx < 16368; idx++)
        {
            uint32_t ncoIdx = m_nco.clk();
            // Multiply float sin/cos by 32767 and cast to int16
            m_ncoBuffer[idx].r = static_cast<int16_t>(m_nco.cosine(ncoIdx) * 1024.0f);
            m_ncoBuffer[idx].i = static_cast<int16_t>(m_nco.sine(ncoIdx) * 1024.0f);
        }

        // Zero out the padding using the struct members
        for (size_t idx = 16368; idx < N; idx++)
        {
            m_ncoBuffer[idx].r = 0;
            m_ncoBuffer[idx].i = 0;
        }

        for (int b = 0; b < numBlocks; b++)
        {
            const kiss_fft_cpx *blockStart = &rawData[b * N];

            // Stage 1: Mix with local NCO to baseband
            complex_mix(m_workspace.data(), blockStart,
                         m_ncoBuffer.data(), N, 2);

            // Stage 2: Forward FFT of the mixed signal
            kiss_fft(m_cfg_fwd, m_workspace.data(), m_workspace.data());

            // Stage 3: Frequency-domain correlation
            complex_mix(m_workspace.data(), m_workspace.data(),
                        currentCodeFft.data(), N, 0);

            // Stage 4: Inverse FFT to get correlation magnitudes
            kiss_fft(m_cfg_inv, m_workspace.data(), m_workspace.data());

            for (size_t idx = 0; idx < N; idx++)
            {
                float r = (float)m_workspace[idx].r;
                float i = (float)m_workspace[idx].i;
                m_accumulatedMag[idx] += std::sqrtf(r * r + i * i);
            }
        }

        float maxMag = 0.0f;
        int peakIndex = 0;
        float sumPower = 0.0f;
        const float norm = 1.0f / (float)(N * numBlocks);

        for (size_t idx = 0; idx < N; idx++)
        {
            float mag = m_accumulatedMag[idx] * norm;
            float pwr = mag * mag;
            sumPower += pwr;

            if (mag > maxMag)
            {
                maxMag = mag;
                peakIndex = (int)idx;
            }
        }

        float peakPower = maxMag * maxMag;
        float avgNoisePower = (sumPower - peakPower) / (float)(N - 1);
        float snr = (avgNoisePower > 1e-12f) ? 10.0f * 
                std::log10f(peakPower / avgNoisePower) : -99.0f;

        // 1. After the peak search finishes finding peakIndex and bestResult
        if (snr > bestResult.snr)
        {
            // Capture the phase of the very last block processed at the peak index
            float peakR = (float)m_workspace[peakIndex].r;
            float peakI = (float)m_workspace[peakIndex].i;
            //atan2 returns radians (-pi to pi)
            float phase = std::atan2f(peakI, peakR);
            // 2. Update bestResult assignment
            bestResult = {bin, peakIndex, maxMag, snr, phase, sampleTick};
        }
    }
    return bestResult;
}