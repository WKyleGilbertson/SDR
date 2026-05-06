#include "ChannelProcessor.h"

// Keep the mapping logic identical to your inline function
inline float mapL1IF(uint8_t m, uint8_t s)
{
    float val = (s == 0) ? 1.0f : -1.0f;
    if (m != 0)
        val *= 3.0f;
    return val;
}

ChannelProcessor::ChannelProcessor(double fs_rate, const AcqResult &init, 
    G2INIT sv)
    : _fs(fs_rate),
      _carrNco(10, (float)fs_rate), // New Tracking Carrier NCO
      _codeNco(0, (float)fs_rate), // New Tracking Code NCO
      _m_sv(sv)
{
    _samplesPerMs = (size_t)(_fs / 1000.0);
    _prn = init.prn;
    _doppler_hz = init.bin * 500.0f;
    double initial_phase = init.codePhase;
    uint32_t whole_chips = (uint32_t)std::floor(initial_phase);
    double fractional_part = initial_phase - (double)whole_chips;

    _codeNco.rotations = whole_chips;
    _codeNco.m_phase   = (uint32_t)(fractional_part * 4294967296.0);

//    _nco.SetFrequency(4.092e6f + _doppler_hz); // Fixed to 4.092 MHz
    _carrFreqBasis = 4.092e6f;
    _codeFreqBasis = 1.023e6f;

    // Initialize Tracking NCOs
    _carrNco.SetFrequency(_carrFreqBasis + _doppler_hz);
    _codeNco.SetFrequency(_codeFreqBasis);

    _codeNco.LoadCACODE(_m_sv.CACODE); // Use the 0/1 chips for NCO
    _codeNco.RakeSpacing(CorrelatorSpacing::Narrow);
    //_codeNco.RakeSpacing(CorrelatorSpacing::halfChip);

    _ca_replica.resize(1023);
    for (int i = 0; i < 1023; i++)
        _ca_replica[i] = (int8_t)_m_sv.CODE[i];

    float Bn_carr = 25.0f;
    //float Bn_carr = 15.0f;
    float zeta = 0.707f;
    float loop_gain = 1.0f; // Simplified gain factor

    float omega_n = Bn_carr * 8.0f * zeta / (4.0f * zeta * zeta + 1.0f);
    _tau1Carr = loop_gain / (omega_n * omega_n);
    _tau2Carr = 2.0f * zeta / omega_n;

    float Bn_code = 2.0f;
    //float Bn_code = 0.5f;
    omega_n = Bn_code * 8.0f * zeta / (4.0f * zeta * zeta + 1.0f);
    _tau1Code = loop_gain / (omega_n * omega_n);
    _tau2Code = 2.0f * zeta / omega_n;
}

CorrRes ChannelProcessor::process(const uint8_t *data, size_t count)
{
    // 1. Declare and initialize 1ms accumulators (fixing the C2065 errors)
    float acc_Ei = 0, acc_Eq = 0; 
    float acc_Pi = 0, acc_Pq = 0; 
    float acc_Li = 0, acc_Lq = 0; 
    std::vector<int8_t> blockSymbols;

    // 2. Block-level accumulators (to return total energy to the UI)
    float block_Pi = 0, block_Pq = 0;
    
    const UnpackEntry *lut = GetLUT_FNHN();

    for (size_t i = 0; i < count; ++i)
    {
        const UnpackEntry &entry = lut[data[i]];

        for (int s = 0; s < 2; ++s)
        {
            const ComplexSample &samp = (s == 0) ? entry.s0 : entry.s1;

            uint32_t carrIdx = _carrNco.clk();
            _codeNco.clk();

            float bb_i = (float)samp.i * _carrNco.cosine(carrIdx);
            float bb_q = (float)samp.q * _carrNco.sine(carrIdx);

            acc_Ei += (bb_i * _codeNco.Early);
            acc_Eq -= (bb_q * _codeNco.Early);
            acc_Pi += (bb_i * _codeNco.Prompt);
            acc_Pq -= (bb_q * _codeNco.Prompt);
            acc_Li += (bb_i * _codeNco.Late);
            acc_Lq -= (bb_q * _codeNco.Late);
        }

        // --- 3. THE 1ms TRIGGER ---
        if ((i + 1) % _samplesPerMs == 0)
        {
            float T = 0.001f; 

            // Carrier Loop
            float carrError = atan2(acc_Pq, acc_Pi) / (2.0f * (float)M_PI);
            float carrNcoUpdate = _oldCarrNco +
                                  (_tau2Carr / _tau1Carr) * (carrError - _oldCarrError) +
                                  (carrError * (T / _tau1Carr));
            _oldCarrNco = carrNcoUpdate;
            _oldCarrError = carrError;
            _carrNco.SetFrequency((_carrFreqBasis + _doppler_hz) - carrNcoUpdate);
            
            float carrierDrift = (_doppler_hz - carrNcoUpdate);
            float aiding = carrierDrift / 1540.0f;

            // Code Loop
            float E2 = (acc_Ei * acc_Ei) + (acc_Eq * acc_Eq);
            float L2 = (acc_Li * acc_Li) + (acc_Lq * acc_Lq);
            float codeError = (E2 - L2) / (E2 + L2);

            codeError *= 10.0f;

            float codeNcoUpdate = _oldCodeNco +
                                  (_tau2Code / _tau1Code) * (codeError - _oldCodeError) +
                                  (codeError * (T / _tau1Code));
            _oldCodeNco = codeNcoUpdate;
            _oldCodeError = codeError;
            _codeNco.SetFrequency(_codeFreqBasis + codeNcoUpdate + aiding);

            // Accumulate for block return
            block_Pi += acc_Pi;
            block_Pq += acc_Pq;

            // Capture the symbol: 1 for positive, -1 for negative
            blockSymbols.push_back((acc_Pi > 0) ? 1 : -1);

            // RESET for next 1ms
            acc_Ei = 0; acc_Eq = 0;
            acc_Pi = 0; acc_Pq = 0;
            acc_Li = 0; acc_Lq = 0;
        }
    }

    double finePhase = (double)_codeNco.m_phase / 4294967296.0;
    _code_phase = (double)_codeNco.rotations + finePhase;

    return {(double)block_Pi, (double)block_Pq, _code_phase, blockSymbols};
}