#include "ChannelProcessor.h"

/* Keep the mapping logic identical to your inline function
inline float mapL1IF(uint8_t m, uint8_t s)
{
    float val = (s == 0) ? 1.0f : -1.0f;
    if (m != 0)
        val *= 3.0f;
    return val;
} */

void ChannelProcessor::resetAccumulators(Accumulators &acc)
{
    acc.Pi = 0;
    acc.Pq = 0;
    acc.Ei = 0;
    acc.Eq = 0;
    acc.Li = 0;
    acc.Lq = 0;
    acc.SEi = 0;
    acc.SEq = 0;
    acc.SLi = 0;
    acc.SLq = 0;
}

void ChannelProcessor::calculateSNR(Accumulators &acc, double &snr)
{
    float pPwr = (float(acc.Pi * acc.Pi + acc.Pq * acc.Pq));
    // Average noise power across Early and Late correlators
    float noisePwr = float((acc.SEi * acc.SEi + acc.SEq * acc.SEq +
                            acc.SLi * acc.SLi + acc.SLq * acc.SLq)) / 4.0f;
    if (noisePwr > 0 && pPwr > 0) // Ensure both are positive
    {
        float instantSNR = 10.0f * log10(pPwr / noisePwr);

        if (snr == -999.0f) {
            snr = instantSNR;
        } else {
            snr = (0.9 * snr) + (0.1f * instantSNR);
        }
    } else if (pPwr <= 0 && noisePwr > 0) {
            snr = (snr == -999.0f) ? -20.0f : (0.8f * snr) + (0.1f * -20.0f); 
        
    }
}

ChannelProcessor::ChannelProcessor(double fs_rate, const AcqResult &init,
                                   G2INIT sv)
    : _fs(fs_rate),
      _carrNco(10, (float)fs_rate), // New Tracking Carrier NCO
      _codeNco(0, (float)fs_rate),  // New Tracking Code NCO
      _m_sv(sv)
{
    for (int i = 0; i < 20; i++)
    {
        _sync.buffer[i] = 0;
        _sync.histograms[i] = 0;
    }
    resetAccumulators(_acc);
    _snr = -999.0f;
    _isLocked = false;
    _rolloverDelayCounter = -1;
    _sync.count = 0;
    _sync.bestOffset = -1;
    _bitAccI = 0.0f;
    _bitAccQ = 0.0f;
    _lastSymbol = 0;
    _samplesPerMs = (size_t)(_fs / 1000.0);
    _prn = init.prn;
    _doppler_hz = init.bin * 500.0f;
    _oldCarrError = 0.0f;
    _oldCarrNco = 0.0f;

    // Rake Initialization
    _codeNco.LoadCACODE(_m_sv.CACODE); // Use the 0/1 chips for NCO
//    _codeNco.RakeSpacing(CorrelatorSpacing::Narrow);
    _codeNco.RakeSpacing(CorrelatorSpacing::halfChip);

    // Handover Logic: How many chips are "in flight" inside 64-bit register
    // before they hit the prompt mask at Bit 32
    float spc = (float)_fs / 1023000.0f;
    int chipTravelDelay = (int)std::round(32.0f / spc);

    double initial_phase = init.codePhase;

    // Add the delay to the rotations so the signal peak is at the Prompt bit
    uint32_t whole_chips = (uint32_t)std::floor(initial_phase) + chipTravelDelay;
    double fractional_part = initial_phase - std::floor(initial_phase);

    _codeNco.rotations = whole_chips % 1023; // wrap at 1 ms boundary
    _codeNco.m_phase = (uint32_t)(fractional_part * 4294967296.0);

    //    _nco.SetFrequency(4.092e6f + _doppler_hz); // Fixed to 4.092 MHz
    _carrFreqBasis = 4.092e6f;
    _codeFreqBasis = 1.023e6f;

    // Initialize Tracking NCOs
    _carrNco.SetFrequency(_carrFreqBasis + _doppler_hz);
    _codeNco.SetFrequency(_codeFreqBasis);

    _ca_replica.resize(1023);
    for (int i = 0; i < 1023; i++)
        _ca_replica[i] = (int8_t)_m_sv.CODE[i];

    _carrLF.Bn = 25.0f;
    _carrLF.zeta = 0.707f;
    _carrLF.gain = 1.0f;
    _carrLF.omega_n = _carrLF.Bn * 8.0f * _carrLF.zeta /
                      (4.0f * _carrLF.zeta * _carrLF.zeta + 1.0f);
    _carrLF.tau1 = _carrLF.gain / (_carrLF.omega_n * _carrLF.omega_n);
    _carrLF.tau2 = 2.0f * _carrLF.zeta / _carrLF.omega_n;

    _codeLF.Bn = 1.0f;
    _codeLF.zeta = 0.707f;
    _codeLF.gain = 1.0f;
    _codeLF.omega_n = _codeLF.Bn * 8.0f * _codeLF.zeta /
                      (4.0f * _codeLF.zeta * _codeLF.zeta + 1.0f);
    _codeLF.tau1 = _codeLF.gain / (_codeLF.omega_n * _codeLF.omega_n);
    _codeLF.tau2 = 2.0f * _codeLF.zeta / _codeLF.omega_n;
}

CorrRes ChannelProcessor::process(const uint8_t *data, size_t count)
{
    std::vector<int8_t> symbols;

    float block_Pi = 0, block_Pq = 0;
    const UnpackEntry *lut = GetLUT_FNHN();

    for (size_t i = 0; i < count; ++i)
    {
        const UnpackEntry &entry = lut[data[i]];
        for (int s = 0; s < 2; ++s)
        {
            const ComplexSample &samp = (s == 0) ? entry.s0 : entry.s1;

            uint32_t oldRot = _codeNco.rotations;
            uint32_t carrIdx = _carrNco.clk();
            _codeNco.clk();
            uint32_t newRot = _codeNco.rotations;

            if (newRot < oldRot)
            {
                _rolloverDelayCounter = 32;
            }

            int32_t bb_i = samp.i * _carrNco.cosine(carrIdx);
            int32_t bb_q = samp.q * _carrNco.sine(carrIdx);

            _acc.Ei += (bb_i * _codeNco.Early);
            _acc.Eq -= (bb_q * _codeNco.Early);
            _acc.Pi += (bb_i * _codeNco.Prompt);
            _acc.Pq -= (bb_q * _codeNco.Prompt);
            _acc.Li += (bb_i * _codeNco.Late);
            _acc.Lq -= (bb_q * _codeNco.Late);
            _acc.SEi += (bb_i * _codeNco.superEarly);
            _acc.SEq -= (bb_q * _codeNco.superEarly);
            _acc.SLi += (bb_i * _codeNco.superLate);
            _acc.SLq -= (bb_q * _codeNco.superLate);

            // if (_rolloverDelayCounter == 0)
            if (_rolloverDelayCounter > 0)
            {
                _rolloverDelayCounter--;
                if (_rolloverDelayCounter == 0)
                {
                    // ... [Keep your Carrier/Code loop filter logic here] ...
                    float T = 0.001f;
                    float carrError = 0.0f;
                    float codeError = 0.0f;
                    float I = (float) _acc.Pi;
                    float Q = (float) _acc.Pq;

                    if (std::abs(I) >= 1e-6f)
                    {
                        carrError = atanf(Q/I);
                    }
                    else
                    {
                        carrError = (Q >= 0.0f) ? 1.570796f : -1.570796f;
                    }

                    float carrNcoUpdate = _oldCarrNco +
                                          (_carrLF.tau2 / _carrLF.tau1) * (carrError - _oldCarrError) +
                                          (carrError * (T / _carrLF.tau1));
                    _oldCarrNco = carrNcoUpdate;
                    _oldCarrError = carrError;

                    float finalFreq = ((_carrFreqBasis + _doppler_hz) - carrNcoUpdate);
                    _carrNco.SetFrequency(finalFreq);
                    _currentCommandedFreq = finalFreq;

                    // --- 2. CODE LOOP (DLL - Delay Locked Loop) ---
                    // Use the "Normalized Early-Minus-Late" discriminator
                    float E = sqrtf(float(_acc.Ei * _acc.Ei + _acc.Eq * _acc.Eq));
                    float L = sqrtf(float(_acc.Li * _acc.Li + _acc.Lq * _acc.Lq));
                    if ((E + L) > 0.0f) {
                    codeError = (E - L) / (E + L);
                    }

                    // If E and L are basically zero, don't update (avoid divide by zero)
                    if (std::isnan(codeError))
                        codeError = 0;

                    float codeNcoUpdate = _oldCodeNco +
                                          (_codeLF.tau2 / _codeLF.tau1) * (codeError - _oldCodeError) +
                                          (codeError * (T / _codeLF.tau1));
                    _oldCodeNco = codeNcoUpdate;
                    _oldCodeError = codeError;

                    // Adjust the Code NCO (1.023 MHz basis + the loop correction)
                    float activeDoppler = finalFreq - _carrFreqBasis;
                    float aiding = activeDoppler / 1540.0f;

                    _codeNco.SetFrequency(_codeFreqBasis + aiding);
                    //  -- 1 ms EPOCH HANDOVER
                    // --- TRACKING TELEMETRY
                    calculateSNR(_acc, _snr);
                    _isLocked = (_snr > 10.f);
                    symbols.push_back((_acc.Pi > 0) ? 1 : -1);

                    // 3. NOW reset the 1ms accumulators for the next Gold Code epoch
                    block_Pi += _acc.Pi;
                    block_Pq += _acc.Pq;
                    // --- Accumulator RESET ---
                    resetAccumulators(_acc);
                }
            }
        }
    }

    double finePhase = (double)_codeNco.m_phase / 4294967296.0;
    _code_phase = (double)_codeNco.rotations + finePhase;
    float dF = _currentCommandedFreq - _carrFreqBasis;

    return {(double)block_Pi, (double)block_Pq, _code_phase, dF, _snr, symbols};
}