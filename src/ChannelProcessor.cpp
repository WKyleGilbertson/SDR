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
      _codeNco(0, (float)fs_rate),  // New Tracking Code NCO
      _m_sv(sv)
{
    for (int i = 0; i < 20; i++)
    {
        _sync.buffer[i] = 0;
        _sync.histograms[i] = 0;
    }
    _sync.count = 0;
    _sync.bestOffset = -1;
    _bitAccI = 0.0f;
    _bitAccQ = 0.0f;
    _lastSymbol = 0;
    _samplesPerMs = (size_t)(_fs / 1000.0);
    _prn = init.prn;
    _doppler_hz = init.bin * 500.0f;

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

    float Bn_carr = 15.0f;
    float zeta = 0.707f;
    float loop_gain = 1.0f; // Simplified gain factor

    float omega_n = Bn_carr * 8.0f * zeta / (4.0f * zeta * zeta + 1.0f);
    _tau1Carr = loop_gain / (omega_n * omega_n);
    _tau2Carr = 2.0f * zeta / omega_n;

    float Bn_code = 1.0f;
    omega_n = Bn_code * 8.0f * zeta / (4.0f * zeta * zeta + 1.0f);
    _tau1Code = loop_gain / (omega_n * omega_n);
    _tau2Code = 2.0f * zeta / omega_n;
}

CorrRes ChannelProcessor::process(const uint8_t *data, size_t count)
{
    float acc_Ei = 0, acc_Eq = 0;
    float acc_Pi = 0, acc_Pq = 0;
    float acc_Li = 0, acc_Lq = 0;
    std::vector<int8_t> blockSymbols;
    std::vector<int8_t> navBits; // <-- ADD THIS to collect 20ms bits in this block

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

            float bb_i = (float)samp.i * _carrNco.cosine(carrIdx);
            float bb_q = (float)samp.q * _carrNco.sine(carrIdx);

            acc_Ei += (bb_i * _codeNco.Early);
            acc_Eq -= (bb_q * _codeNco.Early);
            acc_Pi += (bb_i * _codeNco.Prompt);
            acc_Pq -= (bb_q * _codeNco.Prompt);
            acc_Li += (bb_i * _codeNco.Late);
            acc_Lq -= (bb_q * _codeNco.Late);

            if (_rolloverDelayCounter == 0)
            {
                // ... [Keep your Carrier/Code loop filter logic here] ...
                float T = 0.001f;
                float carrError = atan2f(acc_Pq, acc_Pi) / (2.0f * (float)M_PI);
                if (carrError > 1.570796f)
                    carrError -= M_PI;
                if (carrError < 1.570796f)
                    carrError += M_PI;
                float carrNcoUpdate = _oldCarrNco +
                                      (_tau2Carr / _tau1Carr) * (carrError - _oldCarrError) +
                                      (carrError * (T / _tau1Carr));
                _oldCarrNco = carrNcoUpdate;
                _oldCarrError = carrError;
                _carrNco.SetFrequency((_carrFreqBasis + _doppler_hz) - carrNcoUpdate);

                float carrierDrift = (_doppler_hz - carrNcoUpdate);
                float aiding = carrierDrift / 1540.0f;
                // float phaseError = atanf(acc_Pq / acc_Pi);
                //                float p = phaseError * _tau2Carr / _tau1Carr;
                //                _carrLoopInt += (phaseError * T / _tau1Carr);
                //                float carrW = p + _carrLoopInt;
                //                _carrNco.SetFrequency(_carrFreqBasis + _doppler_hz + (carrW/(2.0f * M_PI)));
                // --- 2. CODE LOOP (DLL - Delay Locked Loop) ---
                // Use the "Normalized Early-Minus-Late" discriminator
                float E = sqrtf(acc_Ei * acc_Ei + acc_Eq * acc_Eq);
                float L = sqrtf(acc_Li * acc_Li + acc_Lq * acc_Lq);
                float codeError = (E - L) / (E + L);

                // If E and L are basically zero, don't update (avoid divide by zero)
                if (std::isnan(codeError))
                    codeError = 0;

                //    float pCode = codeError * _tau2Code / _tau1Code;
                //    _codeLoopInt += (codeError * T / _tau1Code);
                //    float codeW = pCode + _codeLoopInt;
                float codeNcoUpdate = _oldCodeNco +
                                      (_tau2Code / _tau1Code) * (codeError - _oldCodeError) +
                                      (codeError * (T / _tau1Code));
                _oldCodeNco = codeNcoUpdate;
                _oldCodeError = codeError;

                // Adjust the Code NCO (1.023 MHz basis + the loop correction)
                _codeNco.SetFrequency(_codeFreqBasis + aiding);
                // 1. Accumulate the 1ms correlation into the 20ms bit bucket
                // Do this BEFORE resetting acc_Pi
                if (_sync.bestOffset != -1)
                {
                    _bitAccI += acc_Pi;
                    _bitAccQ += acc_Pq;

                    // 2. Check if this millisecond is the end of a 20ms bit
                    if (_sync.count % 20 == _sync.bestOffset)
                    {
                        int8_t navBit = (_bitAccI > 0) ? 1 : -1;
                        navBits.push_back(navBit);

                        _bitAccI = 0; // Reset 20ms bucket
                        _bitAccQ = 0;
                    }
                }

                // --- BIT SYNC & HISTOGRAM UPDATES ---
                int8_t currentSymbol = (acc_Pi > 0) ? 1 : -1;
                blockSymbols.push_back(currentSymbol);

                for (int j = 0; j < 19; j++)
                {
                    _sync.buffer[j] = _sync.buffer[j + 1];
                }
                _sync.buffer[19] = currentSymbol;

                if (_sync.count > 0)
                {
                    int offset = _sync.count % 20;
                    if (currentSymbol != _lastSymbol)
                    {
                        _sync.histograms[offset]++;
                    }
                }
                _lastSymbol = currentSymbol;
                _sync.count++;

                // Histogram re-calc every 200ms
                if (_sync.count % 200 == 0)
                {
                    int maxFlips = -1;
                    int edgeOffset = 0;
                    for (int o = 0; o < 20; o++)
                    {
                        if (_sync.histograms[o] > maxFlips)
                        {
                            maxFlips = _sync.histograms[o];
                            edgeOffset = o;
                        }
                    }
                    _sync.bestOffset = (edgeOffset + 1) % 20;
                    for (int o = 0; o < 20; o++)
                        _sync.histograms[o] = 0;
                }

                // 3. NOW reset the 1ms accumulators for the next Gold Code epoch
                block_Pi += acc_Pi;
                block_Pq += acc_Pq;

                acc_Ei = 0;
                acc_Eq = 0;
                acc_Pi = 0;
                acc_Pq = 0;
                acc_Li = 0;
                acc_Lq = 0;
                _rolloverDelayCounter = -1;
            }

            if (_rolloverDelayCounter > 0)
            {
                _rolloverDelayCounter--;
            }
        }
    }

    double finePhase = (double)_codeNco.m_phase / 4294967296.0;
    _code_phase = (double)_codeNco.rotations + finePhase;

    // Return the structure including the new navBits vector
    return {(double)block_Pi, (double)block_Pq, _code_phase, blockSymbols, navBits};
}