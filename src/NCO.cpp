// Filename: NCO.cpp                                                  2019-02-01
// https://zipcpu.com/dsp/2017/12/09/nco.html

#include "NCO.h"
#include <cstdio>

NCO::NCO(const int lgtblsize, const float m_sample_clk) {
    SAMPLE_RATE = m_sample_clk;
    // We'll use a table 2^(lgtblize) in length.  This is non-negotiable, as the
    // rest of this algorithm depends upon this property.
    m_lglen = lgtblsize;
    m_len = (1 << lgtblsize);

    // m_mask is 1 for any bit used in the index, zero otherwise
    m_mask = m_len - 1;
    //m_table = new float[m_len];

/*    printf("NCO Init: m_len = %d, this = %p\n", m_len, (void*)this);
    if (this == nullptr) { printf("CRITICAL: this is null!\n"); } */

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

void NCO::RakeSpacing(CorrelatorSpacing cs) {
  switch (cs) {
   case CorrelatorSpacing::Narrow:
//    printf("Narrow\n");
    E_mask = 0x0000000000000001;
    P_mask = 0x0000000000000008; // 3
    L_mask = 0x0000000000000040; // 6
    shift  = 3;
    break;
   case CorrelatorSpacing::halfChip:
//    printf("Normal\n");
/*    E_mask = 0x0000000000000001;
    P_mask = 0x0000000000080000;// 19
    L_mask = 0x0000004000000000;// 38
    shift  = 19; */
    E_mask = 0x0000000000000001;
    P_mask = 0x0000000000000100;// 8
    L_mask = 0x0000000000010000;// 16
    shift  = 8;
    break;
   default:
//    printf("No Value %d\n", cs);
      break;
  }  
}

//void NCO::LoadCACODE(int8_t *CODE) {
void NCO::LoadCACODE(uint8_t *CODE) {
  uint16_t i=0;
  for(i = 0; i<1023; i++) {
   CACODE[i] = CODE[i];
  }
}

uint32_t NCO::clk(void) {
    uint32_t index;

    // 1. Advance Phase and check for Code Chip rollover
    if (m_phase + m_dphase < m_phase) // Corrected overflow check
    {
        rotations++;
        if (rotations >= 1023) rotations = 0; 
    }

    // 2. Update the Shift Register with the current Gold Code chip
    EPLreg <<= 1;
    // Ensure we only pull the LSB (0 or 1) from the CACODE table
    EPLreg |= (static_cast<uint64_t>(CACODE[rotations]) & 0x01ULL);

    // 3. Extract Early, Prompt, and Late bits
    // Use the non-zero check (!= 0) to avoid needing specific 'shift' logic
    // This turns a "high" bit into 1 and a "low" bit into -1
    Early  = ((EPLreg & E_mask) != 0) ? 1 : -1;
    Prompt = ((EPLreg & P_mask) != 0) ? 1 : -1;
    Late   = ((EPLreg & L_mask) != 0) ? 1 : -1;

    // 4. Update phase accumulator for Carrier/Table lookup
    m_phase += m_dphase;

    // 5. Generate index for Sine/Cosine LUT
    index = m_phase >> (32 - m_lglen);
    idx = index & m_mask;

    return idx;
}

float NCO::cosine(int32_t idx) {
    return m_costable[idx];
}

float NCO::sine(int32_t idx) {
   return m_sintable[idx]; 
}
