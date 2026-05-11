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
    m_rotations = 0;
}

NCO::~NCO(void) {
    delete[] m_sintable; // On any object deletion, delete the table as well
    delete[] m_costable;
}

void NCO::SetFrequency(float f) {
    // Convert the frequency to a fractional difference in phase
    m_dphase = (int)(f * ONE_ROTATION / SAMPLE_RATE);
    Frequency = f;
}

void NCO::RakeSpacing(CorrelatorSpacing cs) {
    // 1. Calculate samples per chip based on stored sample clock
    // C/A chip rate is 1.023 MHz
    float samplesPerChip = m_sample_clk / 1023000.0f;

    // 2. Anchor Prompt at Bit 32 (The center of the 64-bit register)
    P_mask = 1ULL << 32;

    if (cs == CorrelatorSpacing::halfChip) {
        // 0.5 chip spacing
        shift = (uint8_t)std::round(samplesPerChip * 0.5f);
    } 
    else if (cs == CorrelatorSpacing::Narrow) {
        // 0.1 chip spacing (min 1 sample)
        shift = (uint8_t)std::max(1, (int)std::round(samplesPerChip * 0.1f));
    }

    E_mask = 1ULL << (32 - shift); // "Earlier" in time (entered more recently)
    L_mask = 1ULL << (32 + shift); // "Later" in time (shifted further away)

    // 3. Noise Masks (2.0 chips away)
    // At 16.368MHz, this is exactly 32 samples (Bit 0 and Bit 64)
    int noiseOffset = (int)std::round(samplesPerChip * 2.0f);
    SE_mask = 1ULL << (std::max(0, 32 - noiseOffset));
    SL_mask = 1ULL << (std::min(63, 32 + noiseOffset));
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
    uint32_t rolloverFlag = 0x00000000; // Default: No rollover

    // 1. Advance Phase and check for Code Chip rollover
    if (m_phase + m_dphase < m_phase) 
    {
        m_rotations++;
        if (m_rotations >= 1023) {
            m_rotations = 0;
            rolloverFlag = 0x80000000; // Set top bit high to indicate a 1ms epoch wrap!
        }
    }

    // 2. Update the Shift Register with current Gold Code chip
    EPLreg <<= 1;
    EPLreg |= (static_cast<uint64_t>(CACODE[m_rotations]) & 0x01ULL);

    // 3. Extract Correlator Pipeline Masks
    Early      = ((EPLreg & E_mask) != 0) ? 1 : -1;
    Prompt     = ((EPLreg & P_mask) != 0) ? 1 : -1;
    Late       = ((EPLreg & L_mask) != 0) ? 1 : -1;
    superEarly = ((EPLreg & SE_mask) != 0) ? 1 : -1;
    superLate  = ((EPLreg & SL_mask) != 0) ? 1 : -1; // Assigned and working

    // 4. Update phase accumulator for Carrier/Table lookup
    m_phase += m_dphase;

    // 5. Generate index for Sine/Cosine LUT
    index = m_phase >> (32 - m_lglen);
    idx = index & m_mask;

    // Bitwise OR the look-up index with our top-bit rollover status flag
    return (idx | rolloverFlag); 
}

void NCO::InitializeEPLPipeline(double initialCodePhase, int chipTravelDelay) {
    // Clear out any old history in the shift register
    EPLreg = 0ULL;

    // Calculate the target rotation chip tracking index at handover
    uint32_t base_rotation = (uint32_t)std::floor(initialCodePhase);
    double fractional_part = initialCodePhase - std::floor(initialCodePhase);

    // Set the phase register precisely to match the fractional remainder
    m_phase = (uint32_t)(fractional_part * 4294967296.0);
    m_rotations = (base_rotation + chipTravelDelay) % 1023;

    // Pre-fill the 64-bit shift register looking backwards in time
    // So that the target chip sits exactly at the Prompt mask (Bit 32)
    for (int i = 0; i < 64; ++i) {
        // Calculate historical chip positions wrapping at the 1023 C/A boundary
        int historical_offset = (int)m_rotations - 32 + i;
        while (historical_offset < 0) historical_offset += 1023;
        uint32_t chip_idx = (uint32_t)(historical_offset) % 1023;

        // Shift the bit into place inside the register pipeline
        EPLreg |= (static_cast<uint64_t>(CACODE[chip_idx]) & 0x01ULL) << i;
    }
}