// Filename: NCO.cpp                                                  2019-02-01
// https://zipcpu.com/dsp/2017/12/09/nco.html

#include "NCO.h"

NCO::NCO(const int lgtblsize, const float m_sample_clk) {
    SAMPLE_RATE = m_sample_clk;
    // We'll use a table 2^(lgtblize) in length.  This is non-negotiable, as the
    // rest of this algorithm depends upon this property.
    m_lglen = lgtblsize;
    m_len = (1 << lgtblsize);

    // m_mask is 1 for any bit used in the index, zero otherwise
    m_mask = m_len - 1;
    //m_table = new float[m_len];
    m_sintable = new float[m_len];
    m_costable = new float[m_len];
    for (auto k = 0; k < m_len; k++) {
        //m_table[k] = sin(2.0 * M_PI * k / (double) m_len);
        m_sintable[k] = (float) sin(2.0 * M_PI * k / (double) m_len);
        m_costable[k] = (float) cos(2.0 * M_PI * k / (double) m_len);
    }
    // m_phase is the variable holding our PHI[n] function from above.
    // We'll initialize our initial phase and frequency to zero
    m_phase = 0;
    m_dphase = 0;
//    m_bias = pow(2,32)*IF/m_sample_clk; // This is not used for anything.
    rotations = 0;
}

NCO::~NCO(void) {
    delete[] m_sintable; // On any object deletion, delete the table as well
    delete[] m_costable;
}

// Adjust the sample rate for your implementation as necessary
//const	float	SAMPLE_RATE= 1.0;
//const float SAMPLE_RATE = 38.192e6; // Moved to header file
//const float ONE_ROTATION = 2.0 * (1u << (sizeof(unsigned) * 8 - 1));

void NCO::SetFrequency(float f) {
    // Convert the frequency to a fractional difference in phase
    m_dphase = (int)(f * ONE_ROTATION / SAMPLE_RATE);
    Frequency = f;
}

void NCO::LoadCACODE(int8_t *CODE) {
  uint16_t i=0;
  for(i = 0; i<1023; i++) {
   CACODE[i] = CODE[i];
  }
}

uint32_t NCO::clk(void) {
//float NCO::operator()(void) {
    uint32_t index;
    // Increment the phase by an amount dictated by our frequency
    // m_phase was our PHI[n] value above
    //if (m_phase + m_dphase < m_dphase) printf("Overflow");
    if (m_phase + m_dphase < m_dphase) 
    {
        rotations++;
        rotations %= 1023;
        //EPLreg |= 0x01;
    }
    m_phase += m_dphase; // PHI[n] = PHI[n-1] + (2^32 * f/fs)
    // Grab the top m_lglen bits of this phase word
    index = m_phase >> ((sizeof(uint32_t) * 8) - m_lglen);
    // Insist that this index be found within 0... (m_len-1)
    index &= m_mask;
    idx = index;
    // Finally return the table lookup value
    return idx;
}

float NCO::cosine(int32_t idx) {
    return m_costable[idx];
}

float NCO::sine(int32_t idx) {
   return m_sintable[idx]; 
}
