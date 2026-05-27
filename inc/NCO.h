// Filename: NCO.h                                                    2026-04-12
//

#pragma once
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <algorithm>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

enum CorrelatorSpacing {halfChip, Narrow};

class NCO {
public:
    int8_t Early = 0, Prompt = 0, Late = 0;
    int8_t superEarly = 0, superLate = 0;
    NCO(const int lgtblsize, const float m_sample_clk);
    ~NCO(void);
    void SetFrequency(float f);
    void RakeSpacing(CorrelatorSpacing cs);
    void LoadCACODE(uint8_t *CODE); // DSP conversion happens in Rake
    uint32_t clk(void);
    float cosine(int32_t idx) const {return m_costable[idx];}
    float sine(int32_t idx) const {return m_sintable[idx];}
    uint32_t getPhase() const {return m_phase;}
    uint32_t getRotations() const {return m_rotations;}
    uint32_t getMask() const {return m_mask;}
    void NCO::InitializeEPLPipeline(double initialCodePhase, int chipTravelDelay);
private:
    const float m_ONE_ROTATION = (float) 2.0 * (1u << 31);
    uint32_t m_lglen, m_len, m_mask, m_phase, m_dphase, m_bias, idx;
    uint16_t m_rotations = 0;
    uint64_t EPLreg = 0ULL;
    int8_t CACODE[1023];
    float * m_sintable = nullptr;
    float * m_costable = nullptr;
    float m_sample_clk, m_frequency;
    uint64_t E_mask, P_mask, L_mask, SE_mask, SL_mask;
    uint8_t shift;
};
