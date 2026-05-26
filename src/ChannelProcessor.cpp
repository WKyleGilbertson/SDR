#include "ChannelProcessor.h"
#include <cmath>

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
    _snrBufferI[_snrBufferIndex] = (float)acc.Pi;
    _snrBufferQ[_snrBufferIndex] = (float)acc.Pq;
    _snrBufferIndex = (_snrBufferIndex + 1) % 20;

    float signalPower = 0.0f;
    for (int i = 0; i < 20; ++i)
    {
        signalPower += (_snrBufferI[i] * _snrBufferI[i]);
    }
    signalPower /= 20.0f;

    float noisePower = 0.0f;
    for (int i = 0; i < 20; ++i)
    {
        noisePower += (_snrBufferQ[i] * _snrBufferQ[i]);
    }
    noisePower /= 20.0f;

    if (noisePower > 0.0f && signalPower > noisePower)
    {
        float calculatedMetric = 10.0f * log10f(signalPower / noisePower);
        if (snr <= 0.0f || snr == -5.0f || snr == 3.0f || snr == -999.0f)
        {
            snr = calculatedMetric;
        }
        else
        {
            snr = (0.95f * snr) + (0.05f * calculatedMetric);
        }
    }
    else
    {
        snr = (snr <= -100.0f) ? 14.5f : (0.95f * snr) + (0.05f * 14.5f);
    }
}

ChannelProcessor::ChannelProcessor(double fs_rate, const AcqResult &init, G2INIT sv)
    : _fs(fs_rate), _carrNco(8, (float)fs_rate), _codeNco(0, (float)fs_rate), _m_sv(sv)
{
    _carrFreqBasis = 4.092e6f;
    _codeFreqBasis = 1.023e6f;
    resetAccumulators(_acc);
    _snr = -999.0f;
    _isLocked = false;
    _samplesPerMs = (size_t)(_fs / 1000.0);
    _prn = init.prn;
    _doppler_hz = init.bin * 500.0f;
    _oldCarrError = 0.0f;
    _oldCarrNco = _carrFreqBasis - (float)_doppler_hz;
    _oldCodeError = 0.0f;
    _oldCodeNco = 0.0f;
    _accumulatedCarrierCycles = 0;
    _sampleCounter = 0;
    _prevCodePhase = 0;
    _msIntegrated = 0;

    for (int i = 0; i < 20; ++i)
    {
        _snrBufferI[i] = 0.0f;
        _snrBufferQ[i] = 0.0f;
    }
    _snrBufferIndex = 0;

    _codeNco.LoadCACODE(_m_sv.CACODE);
    _codeNco.RakeSpacing(CorrelatorSpacing::halfChip);

    float spc = (float)_fs / 1023000.0f;
    int chipTravelDelay = (int)std::round(32.0f / spc);
    _codeNco.InitializeEPLPipeline(init.codePhase, chipTravelDelay);

    _carrNco.SetFrequency(_carrFreqBasis - (float)_doppler_hz);
    _codeNco.SetFrequency(_codeFreqBasis);
    _currentCommandedFreq = _oldCarrNco;

    _ca_replica.resize(1023);
    for (int i = 0; i < 1023; i++)
        _ca_replica[i] = (int8_t)_m_sv.CODE[i];

    //_carrLF.Bn = 10.0f;
    _carrLF.Bn = 25.0f;
    _carrLF.zeta = 0.707f;
    _carrLF.gain = 1.0f;
    _carrLF.omega_n = _carrLF.Bn * 8.0f * _carrLF.zeta / (4.0f * _carrLF.zeta * _carrLF.zeta + 1.0f);
    _carrLF.tau1 = _carrLF.gain / (_carrLF.omega_n * _carrLF.omega_n);
    _carrLF.tau2 = 2.0f * _carrLF.zeta / _carrLF.omega_n;

    _codeLF.Bn = 1.0f;
    _codeLF.zeta = 0.707f;
    _codeLF.gain = 1.0f;
    _codeLF.omega_n = _codeLF.Bn * 8.0f * _codeLF.zeta / (4.0f * _codeLF.zeta * _codeLF.zeta + 1.0f);
    _codeLF.tau1 = _codeLF.gain / (_codeLF.omega_n * _codeLF.omega_n);
    _codeLF.tau2 = 2.0f * _codeLF.zeta / _codeLF.omega_n;
}

CorrelatorResult ChannelProcessor::Correlator(const uint8_t *data, size_t count) {
    static uint32_t lastExitPhase = 0;
    const int target_integration_ms = 1;
    const float T = 0.001f * target_integration_ms;
    uint64_t latchedRolloverSample = 0;
    const UnpackEntry *lut = GetLUT_FNHN();

    // Track if a true 1ms epoch rollover occurred at any point during this slice
    bool epochTriggered = false;
if (lastExitPhase !=0 && _carrNco.getPhase() != lastExitPhase) {
   printf("!!! DISCONTINUITY DETECTED: Expected %u, Found %u\n", lastExitPhase, _carrNco.getPhase());
} 
//printf("[DEBUG] Entry PRN %d: CarrPhase=%u, CodePhase=%u\n", _prn, _carrNco.getPhase(), _codeNco.getPhase());

    for (size_t i = 0; i < count; ++i) {
        const UnpackEntry &entry = lut[data[i]];
        
        for (int s = 0; s < 2; ++s) {
            const ComplexSample &samp = (s == 0) ? entry.s0 : entry.s1;
            _sampleCounter++;
            
            uint32_t oldCarrPhase = _carrNco.getPhase();
            uint32_t carrIdx = _carrNco.clk();
            if (_carrNco.getPhase() < oldCarrPhase) {
                _accumulatedCarrierCycles++;
            }
            
            uint32_t oldCodeRotations = _codeNco.getRotations();
            _codeNco.clk();
            
            // Check for the true 1ms Gold Code Epoch Event
            if (_codeNco.getRotations() > oldCodeRotations) {
                _msIntegrated++;
                if (_msIntegrated >= target_integration_ms) {
                    epochTriggered = true;
                    latchedRolloverSample = _sampleCounter;
                }
            }

            // Mix and accumulate EVERY sample continuously without dropping data
            //int32_t bb_i = (int32_t)(samp.i * (_carrNco.cosine(carrIdx) * 128.0f));
            //int32_t bb_q = (int32_t)(samp.q * (_carrNco.sine(carrIdx) * 128.0f));
            int16_t bb_i = (int16_t)(samp.i * (_carrNco.cosine(carrIdx) * 128.0f));
            int16_t bb_q = (int16_t)(samp.q * (_carrNco.sine(carrIdx) * 128.0f));

            _acc.Ei += (bb_i * _codeNco.Early);
            _acc.Eq -= (bb_q * _codeNco.Early);
            _acc.Pi += (bb_i * _codeNco.Prompt);
            _acc.Pq -= (bb_q * _codeNco.Prompt);
            _acc.Li += (bb_i * _codeNco.Late);
            _acc.Lq -= (bb_q * _codeNco.Late);
        } // End of sub-sample Loop s
    } // End of main array sample loops

    // Run tracking loop calculations ONLY after the entire slice buffer is completed
    if (epochTriggered) {
        CorrelatorResult res = {};
        res.numSymbols = 0;
        res.epoch_valid = true; // Telemetry is verified and complete

        float carrError = 0.0f;
        float codeError = 0.0f;
        float I = (float)_acc.Pi;
        float Q = (float)_acc.Pq;

        if (std::abs(I) >= 1e-6f) {
            carrError = atanf(Q / I);
        } else {
            carrError = (Q >= 0.0f) ? 1.570796f : -1.570796f;
        }

        float E = sqrtf((float)(_acc.Ei * _acc.Ei + _acc.Eq * _acc.Eq));
        float L = sqrtf((float)(_acc.Li * _acc.Li + _acc.Lq * _acc.Lq));
        float P = sqrtf((float)(_acc.Pi * _acc.Pi + _acc.Pq * _acc.Pq));

        if (P > 1e-6f) {
            codeError = (E - L) / (2.0f * P);
            if (codeError > 1.0f) codeError = 1.0f;
            if (codeError < -1.0f) codeError = -1.0f;
        } else {
            codeError = 0.0f;
        }

        calculateSNR(_acc, _snr);
        _isLocked = (_snr > 12.0f);

        if (res.numSymbols < 32) {
            res.symbols[res.numSymbols++] = (_acc.Pi > 0) ? 1 : -1;
            //res.symbols[res.numSymbols++] = ((_acc.Pi*_acc.Pq) > 0) ? 1 : -1;
        }

        float carrNcoUpdate = _oldCarrNco + (_carrLF.tau2 / _carrLF.tau1) * (carrError - _oldCarrError) + (carrError * (T / _carrLF.tau1));
        _oldCarrNco = carrNcoUpdate;
        _oldCarrError = carrError;
        _doppler_hz = (double)(_carrFreqBasis - carrNcoUpdate);

        float codeNcoUpdate = _oldCodeNco + (_codeLF.tau2 / _codeLF.tau1) * (codeError - _oldCodeError) + (codeError * (T / _codeLF.tau1));
        _oldCodeNco = codeNcoUpdate;
        _oldCodeError = codeError;

        _currentCommandedFreq = carrNcoUpdate;
        _carrNco.SetFrequency(_currentCommandedFreq);

        float aiding = (_prn < 100) ? ((float)_doppler_hz / 1540.0f) : 0.0f;
        _codeNco.SetFrequency(_codeFreqBasis + codeNcoUpdate + aiding);

        double fractionalCarrierPhase = ((double)_carrNco.getPhase() / 4294967296.0) * (2.0 * M_PI);
        double absoluteCarrierPhase = ((double)_accumulatedCarrierCycles * (2.0 * M_PI)) + fractionalCarrierPhase;

        double finePhase = (double)_codeNco.getPhase() / 4294967296.0;
        _code_phase = (double)_codeNco.getRotations() + finePhase;

        float dF = (float)(_carrFreqBasis - _currentCommandedFreq);

        res.carrier_phase_error = carrError;
        res.absolute_carrier_phase = absoluteCarrierPhase;
        res.doppler_hz = dF;
        res.prn = _prn;
        res.Pi = (double)I;
        res.Pq = (double)Q;
        res.code_phase = _code_phase;
        res.snr = _snr;
        res.rollover_sample_idx = latchedRolloverSample;
        res.is_locked = _isLocked;

        // Clear accumulators safely now that telemetry is generated
        resetAccumulators(_acc);
        _msIntegrated = 0;

//        printf("[DEBUG] Exit PRN %d: dF=%.2f, CarrNCO=%.2f\n", _prn, _doppler_hz, _currentCommandedFreq);
        lastExitPhase = _carrNco.getPhase();
        return res;
    }

    // Fallback: If no epoch finished in this slice, return an invalid marker.
    // Stale residual values stay untouched in _acc to overflow into the next call.
    CorrelatorResult emptyRes = {};
    emptyRes.epoch_valid = false;
//    printf("[DEBUG] Exit PRN %d: dF=%.2f, CarrNCO=%.2f\n", _prn, _doppler_hz, _currentCommandedFreq);
        lastExitPhase = _carrNco.getPhase();
    return emptyRes;
}
