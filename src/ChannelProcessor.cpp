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

    const UnpackEntry *lut = GetLUT_FNHN();

    for (size_t i = 0; i < count; ++i)
    {
        // 2. Single lookup for both samples in the byte
        const UnpackEntry &entry = lut[data[i]];

        // --- SAMPLE 0 (High Nibble) ---
        {
            uint32_t nco_idx = _nco.clk();
            int8_t code_val = _ca_replica[(int)_code_phase % 1023];

            // Access directly from LUT entry (replaces mapL1IF calls)
            acc_i += ((float)entry.s0.i * code_val * _nco.cosine(nco_idx));
            acc_q -= ((float)entry.s0.q * code_val * _nco.sine(nco_idx));

            _code_phase += chips_per_sample;
        }

        // --- SAMPLE 1 (Low Nibble) ---
        {
            uint32_t nco_idx = _nco.clk();
            int8_t code_val = _ca_replica[(int)_code_phase % 1023];

            // Access directly from LUT entry
            acc_i += ((float)entry.s1.i * code_val * _nco.cosine(nco_idx));
            acc_q -= ((float)entry.s1.q * code_val * _nco.sine(nco_idx));

            _code_phase += chips_per_sample;
        }

        // Epoch wrap-around logic...
        if (_code_phase >= 1023.0)
            _code_phase -= 1023.0;
    }
    return {(double)acc_i, (double)acc_q, (double)_code_phase};
}