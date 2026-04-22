// Filename: NCO.h                                                    2026-04-12
//

#pragma once
#include <cmath>
#include <cstdint>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

class NCO {
public:
    const float ONE_ROTATION = (float) 2.0 * (1u << 31);
    float SAMPLE_RATE, Frequency;
    uint32_t m_lglen, m_len, m_mask, m_phase, m_dphase, m_bias, k, idx;
    float * m_sintable, * m_costable, m_sample_clk;
    uint16_t rotations;
    int8_t CACODE[1023];
    float * m_table;
    NCO(const int lgtblsize, const float m_sample_clk);
    ~NCO(void);
    void SetFrequency(float f);
    void LoadCACODE(int8_t *CODE);
    uint32_t clk(void);
    float cosine(int32_t idx);
    float sine(int32_t idx);
//private:

};
