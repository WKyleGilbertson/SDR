#include "ChannelProcessor.h"

// Keep the mapping logic identical to your inline function
inline float mapL1IF(uint8_t m, uint8_t s)
{
    float val = (s == 0) ? 1.0f : -1.0f;
    if (m != 0)
        val *= 3.0f;
    return val;
}

ChannelProcessor::ChannelProcessor(double fs_rate, const AcqResult &init)
    : _fs(fs_rate), _nco(10, (float)fs_rate)
{
    _prn = init.prn;
    _code_phase = init.codePhase; // Ensure this matches struct field name

    _doppler_hz = init.bin * 500.0f;
    _nco.SetFrequency(4.092e6f + _doppler_hz); // Fixed to 4.092 MHz

    G2INIT sv(_prn, 0);
    _ca_replica.resize(1023);
    for (int i = 0; i < 1023; i++)
        _ca_replica[i] = (int8_t)sv.CODE[i];
}

CorrRes ChannelProcessor::process(const uint8_t *data, size_t count)
{
    float acc_i = 0.0f;
    float acc_q = 0.0f;
    float current_code_phase = 0.0f;
    const double code_doppler = _doppler_hz / 1540.0;
    const double chips_per_sample = (1023000.0 + code_doppler) / _fs;

    for (size_t i = 0; i < count; ++i)
    {
        uint8_t b = data[i];

        // --- SAMPLE 0 (High Nibble: Bits 7:4) ---
        {
            uint32_t nco_idx = _nco.clk();
            int8_t code_val = _ca_replica[(int)_code_phase % 1023];

            // Unpack Sample 0 (FNHN)
            float i0 = mapL1IF((b >> 6) & 1, (b >> 7) & 1); // Q0_s is bit 7, Q0_m is bit 6?
            float q0 = mapL1IF((b >> 4) & 1, (b >> 5) & 1); // No, looking at your comment:
            // | Q0_s (7) | Q0_m (6) | I0_s (5) | I0_m (4) |

            float real_sample = mapL1IF((b >> 4) & 1, (b >> 5) & 1); // I0_m, I0_s
            float imag_sample = mapL1IF((b >> 6) & 1, (b >> 7) & 1); // Q0_m, Q0_s

            // Despread & Mix
            acc_i += (real_sample * code_val * _nco.cosine(nco_idx));
            acc_q -= (imag_sample * code_val * _nco.sine(nco_idx));

            _code_phase += chips_per_sample;
        }

        // --- SAMPLE 1 (Low Nibble: Bits 3:0) ---
        {
            uint32_t nco_idx = _nco.clk();
            int8_t code_val = _ca_replica[(int)_code_phase % 1023];

            // Unpack Sample 1 (FNHN)
            // | Q1_s (3) | Q1_m (2) | I1_s (1) | I1_m (0) |
            float real_sample = mapL1IF((b >> 0) & 1, (b >> 1) & 1); // I1_m, I1_s
            float imag_sample = mapL1IF((b >> 2) & 1, (b >> 3) & 1); // Q1_m, Q1_s

            acc_i += (real_sample * code_val * _nco.cosine(nco_idx));
            acc_q -= (imag_sample * code_val * _nco.sine(nco_idx));

            _code_phase += chips_per_sample;
        }

        // Ensure we are always within 1ms of C/A code
        while (_code_phase >= 1023.0)
            _code_phase -= 1023.0;
        while (_code_phase < 0.0)
            _code_phase += 1023.0;
        // printf(" [DEBUG INTERNAL CODE: %8.3f] ", this->_code_phase);
    }
    return {(double)acc_i, (double)acc_q, (double)_code_phase};
}