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
    : _fs(fs_rate),
      _nco(10, (float)fs_rate),    // Old NCO for PCS
      _carrNco(5, (float)fs_rate), // New Tracking Carrier NCO
      _codeNco(5, (float)fs_rate) // New Tracking Code NCO
{
    _prn = init.prn;
    _code_phase = init.codePhase; // Ensure this matches struct field name
    _doppler_hz = init.bin * 500.0f;

    _nco.SetFrequency(4.092e6f + _doppler_hz); // Fixed to 4.092 MHz
    _carrFreqBasis = 4.092e6f;
    _codeFreqBasis = 1.023e6f;

    // Initialize Tracking NCOs
    _carrNco.SetFrequency(_carrFreqBasis + _doppler_hz);
    _codeNco.SetFrequency(_codeFreqBasis);

    // G2INIT sv(_prn, 0);
    G2INIT sv(_prn, (uint16_t)_code_phase);

    _codeNco.LoadCACODE(sv.CACODE); // Use the 0/1 chips for NCO
    _codeNco.RakeSpacing(CorrelatorSpacing::halfChip);

    _ca_replica.resize(1023);
    for (int i = 0; i < 1023; i++)
        _ca_replica[i] = (int8_t)sv.CODE[i];

    float Bn_carr = 25.0f;
    float zeta = 0.707f;
    float loop_gain = 1.0f; // Simplified gain factor

    float omega_n = Bn_carr * 8.0f * zeta / (4.0f * zeta * zeta + 1.0f);
    _tau1Carr = loop_gain / (omega_n * omega_n);
    _tau2Carr = 2.0f * zeta / omega_n;

    float Bn_code = 2.0f;
    omega_n = Bn_code * 8.0f * zeta / (4.0f * zeta * zeta + 1.0f);
    _tau1Code = loop_gain / (omega_n * omega_n);
    _tau2Code = 2.0f * zeta / omega_n;
}

CorrRes ChannelProcessor::process(const uint8_t *data, size_t count)
{
    float acc_Ei = 0, acc_Eq = 0; // Early
    float acc_Pi = 0, acc_Pq = 0; // Prompt
    float acc_Li = 0, acc_Lq = 0; // Late

    const UnpackEntry *lut = GetLUT_FNHN(); // Get the pre-baked FNHN LUT

    for (size_t i = 0; i < count; ++i)
    {
        const UnpackEntry &entry = lut[data[i]]; // Fetch byte once

        // Process two samples (nibbles) per byte without duplicating code
        for (int s = 0; s < 2; ++s)
        {
            const ComplexSample &samp = (s == 0) ? entry.s0 : entry.s1;

            // 1. Advance both NCOs
            uint32_t carrIdx = _carrNco.clk();
            _codeNco.clk(); // Updates rotations and EPLreg

            // 2. Mix to Baseband using lookup tables
            float bb_i = (float)samp.i * _carrNco.cosine(carrIdx);
            float bb_q = (float)samp.q * _carrNco.sine(carrIdx);

            // 3. Apply the Rake (E/P/L picks bits from EPLreg shifted by sample clock)
            acc_Ei += (bb_i * _codeNco.Early);
            acc_Eq -= (bb_q * _codeNco.Early);
            acc_Pi += (bb_i * _codeNco.Prompt);
            acc_Pq -= (bb_q * _codeNco.Prompt);
            acc_Li += (bb_i * _codeNco.Late);
            acc_Lq -= (bb_q * _codeNco.Late);
        }
    }

    // --- End of 1ms Block: Apply TrkBST Discriminators ---
    float T = 0.001f; // 1ms update interval

    // 1. Carrier Loop Update (Bilinear Transform)
    float carrError = atan2(acc_Pq, acc_Pi) / (2.0f * (float)M_PI);

    float carrNcoUpdate = _oldCarrNco +
                          (_tau2Carr / _tau1Carr) * (carrError - _oldCarrError) +
                          (carrError * (T / _tau1Carr));

    _oldCarrNco = carrNcoUpdate;
    _oldCarrError = carrError;

    // Adjust frequency: Basis + Doppler - Filter Output
    _carrNco.SetFrequency((_carrFreqBasis + _doppler_hz) - carrNcoUpdate);

    // 2. Code Loop Update (Bilinear Transform)
    float E2 = (acc_Ei * acc_Ei) + (acc_Eq * acc_Eq);
    float L2 = (acc_Li * acc_Li) + (acc_Lq * acc_Lq);
    float codeError = (E2 - L2) / (E2 + L2);

    float codeNcoUpdate = _oldCodeNco +
                          (_tau2Code / _tau1Code) * (codeError - _oldCodeError) +
                          (codeError * (T / _tau1Code));

    _oldCodeNco = codeNcoUpdate;
    _oldCodeError = codeError;

    // Adjust frequency: Basis - Filter Output
    _codeNco.SetFrequency(_codeFreqBasis - codeNcoUpdate);

    // 3. High-Precision Code Phase Calculation
    // Combine coarse rotations (0-1022) with fine NCO phase (0-1.0)
    double finePhase = (double)_codeNco.m_phase / 4294967296.0;
    _code_phase = (double)_codeNco.rotations + finePhase;

    return {(double)acc_Pi, (double)acc_Pq, _code_phase};
}