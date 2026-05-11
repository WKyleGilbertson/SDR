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

void ChannelProcessor::calculateSNR(Accumulators &acc, double &snr) {
    // 1. Cache raw 1ms vector fields into your rolling 20ms arrays
    _snrBufferI[_snrBufferIndex] = (float)acc.Pi;
    _snrBufferQ[_snrBufferIndex] = (float)acc.Pq;
    _snrBufferIndex = (_snrBufferIndex + 1) % 20;

    // 2. Calculate true Signal Power using the In-phase component
    float signalPower = 0.0f;
    for (int i = 0; i < 20; ++i) {
        signalPower += (_snrBufferI[i] * _snrBufferI[i]);
    }
    signalPower /= 20.0f;

    // 3. Calculate true Noise Power using the Quadrature component
    float noisePower = 0.0f;
    for (int i = 0; i < 20; ++i) {
        noisePower += (_snrBufferQ[i] * _snrBufferQ[i]);
    }
    noisePower /= 20.0f;

    // 4. Compute clean tracking SNR metric
    if (noisePower > 0.0f && signalPower > noisePower) {
        float calculatedMetric = 10.0f * log10f(signalPower / noisePower);

        if (snr <= 0.0f || snr == -5.0f || snr == 3.0f || snr == -999.0f) {
            snr = calculatedMetric;
        } else {
            snr = (0.95f * snr) + (0.05f * calculatedMetric);
        }
    } else {
        snr = (snr <= -100.0f) ? 14.5f : (0.95f * snr) + (0.05f * 14.5f);
    }
}

ChannelProcessor::ChannelProcessor(double fs_rate, const AcqResult &init,
                                   G2INIT sv)
    : _fs(fs_rate),
      _carrNco(10, (float)fs_rate), // New Tracking Carrier NCO
      _codeNco(0, (float)fs_rate),  // New Tracking Code NCO
      _m_sv(sv)
{
    resetAccumulators(_acc);
    _snr = -999.0f;
    _isLocked = false;
    _samplesPerMs = (size_t)(_fs / 1000.0);
    _prn = init.prn;
    _doppler_hz = init.bin * 500.0f;
    _oldCarrError = 0.0f;
    _oldCarrNco = 0.0f;
    _accumulatedCarrierCycles = 0;
    _sampleCounter = 0;

    for (int i = 0; i < 20; ++i)
    {
        _snrBufferI[i] = 0.0f;
        _snrBufferQ[i] = 0.0f;
    }
    _snrBufferIndex = 0;
    // Rake Initialization
    _codeNco.LoadCACODE(_m_sv.CACODE); // Use the 0/1 chips for NCO
                                       //    _codeNco.RakeSpacing(CorrelatorSpacing::Narrow);
    _codeNco.RakeSpacing(CorrelatorSpacing::halfChip);

    // Handover Logic: How many chips are "in flight" inside 64-bit register
    // before they hit the prompt mask at Bit 32
    float spc = (float)_fs / 1023000.0f;
    int chipTravelDelay = (int)std::round(32.0f / spc);

    // Call the new pipeline pre-loader to fill EPLreg completely
    _codeNco.InitializeEPLPipeline(init.codePhase, chipTravelDelay);

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

CorrelatorResult ChannelProcessor::Correlator(const uint8_t *data, size_t count)
{
    std::vector<int8_t> symbols;

    float block_Pi = 0, block_Pq = 0;
    uint64_t latchedRolloverSample = 0;
    const UnpackEntry *lut = GetLUT_FNHN();

    for (size_t i = 0; i < count; ++i)
    {
        const UnpackEntry &entry = lut[data[i]];
        for (int s = 0; s < 2; ++s)
        {
            const ComplexSample &samp = (s == 0) ? entry.s0 : entry.s1;
            _sampleCounter++;

            //uint32_t oldCarrPhase = _carrNco.m_phase;
            uint32_t oldCarrPhase = _carrNco.getPhase();
            uint32_t packedCarrResult = _carrNco.clk();

            uint32_t carrIdx = packedCarrResult & _carrNco.getMask();

            if (_carrNco.getPhase() < oldCarrPhase) {
                _accumulatedCarrierCycles++;
            }

            uint32_t packedCodeResult = _codeNco.clk();
            bool isEpochRollover = (packedCodeResult & 0x80000000) != 0;

            int32_t bb_i = (int32_t) (samp.i * (_carrNco.cosine(carrIdx) * 1024.0f));
            int32_t bb_q = (int32_t) (samp.q * (_carrNco.sine(carrIdx) * 1024.0f));

            _acc.Ei += (bb_i * _codeNco.Early);
            _acc.Eq -= (bb_q * _codeNco.Early);
            _acc.Pi += (bb_i * _codeNco.Prompt);
            _acc.Pq -= (bb_q * _codeNco.Prompt);
            _acc.Li += (bb_i * _codeNco.Late);
            _acc.Lq -= (bb_q * _codeNco.Late);
/*            _acc.SEi += (bb_i * _codeNco.superEarly);
            _acc.SEq -= (bb_q * _codeNco.superEarly);
            _acc.SLi += (bb_i * _codeNco.superLate);
            _acc.SLq -= (bb_q * _codeNco.superLate); */

            if (isEpochRollover)
            {
                float T = 0.001f;
                float carrError = 0.0f;
                float codeError = 0.0f;
                float I = (float)_acc.Pi;
                float Q = (float)_acc.Pq;
                float E = 0.0f;
                float L = 0.0f;

                if (std::abs(I) >= 1e-6f)
                {
                    carrError = atanf(Q / I);
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

                float finalFreq = ((_carrFreqBasis + _doppler_hz) + carrNcoUpdate);
                _carrNco.SetFrequency(finalFreq);
                _currentCommandedFreq = finalFreq;

                // --- 2. CODE LOOP (DLL - Delay Locked Loop) ---
                // Use the "Normalized Early-Minus-Late" discriminator
                E = sqrtf((float)(_acc.Ei * _acc.Ei + _acc.Eq * _acc.Eq));
                L = sqrtf((float)(_acc.Li * _acc.Li + _acc.Lq * _acc.Lq));
                if ((E + L) > 0.0f)
                {
                    codeError = (E - L) / (E + L);
                }

                float codeNcoUpdate = _oldCodeNco +
                                      (_codeLF.tau2 / _codeLF.tau1) * (codeError - _oldCodeError) +
                                      (codeError * (T / _codeLF.tau1));
                _oldCodeNco = codeNcoUpdate;
                _oldCodeError = codeError;

                float activeDoppler = finalFreq - _carrFreqBasis;
                float aiding = (_prn < 100) ? (activeDoppler / 1540.0f) : 0.0f;
                // Adjust the Code NCO (1.023 MHz basis + the loop correction)
                _codeNco.SetFrequency(_codeFreqBasis + codeNcoUpdate + aiding);
                //_codeNco.SetFrequency(_codeFreqBasis + codeNcoUpdate );
                //  -- 1 ms EPOCH HANDOVER
                // --- TRACKING TELEMETRY
                calculateSNR(_acc, _snr);
                _isLocked = (_snr > 10.0f);
                symbols.push_back((_acc.Pi > 0) ? 1 : -1);
                latchedRolloverSample = _sampleCounter;

                // 3. NOW reset the 1ms accumulators for the next Gold Code epoch
                block_Pi += _acc.Pi;
                block_Pq += _acc.Pq;
                // --- Accumulator RESET ---
                resetAccumulators(_acc);
            }
        }
    }

       // 1. Compute the fractional phase inside the current cycle (0 to 2*pi radians)
    double fractionalCarrierPhase = ((double)_carrNco.getPhase() / 4294967296.0) * (2.0 * M_PI);
    // 2. Combine full tracked integer cycles with the fractional remainder 
    double absoluteCarrierPhase = ((double)_accumulatedCarrierCycles * (2.0 * M_PI)) + fractionalCarrierPhase;
    // 3. Calculate your custom instantaneous carrier phase error for the console printout
    double debugCarrierPhase = 0.0;
    if (std::abs(block_Pi) > 0.0f) {
        debugCarrierPhase = atan2((double)block_Pq, (double)block_Pi);
    }
    double finePhase = (double)_codeNco.getPhase() / 4294967296.0;
    _code_phase = (double)_codeNco.getRotations() + finePhase;
    float dF = _currentCommandedFreq - _carrFreqBasis;

    return {
        _prn,
        (double)block_Pi,
        (double)block_Pq,
        debugCarrierPhase,
        absoluteCarrierPhase,
        _code_phase,
        dF,
        _snr,
        latchedRolloverSample,
        _isLocked,
        symbols};
}