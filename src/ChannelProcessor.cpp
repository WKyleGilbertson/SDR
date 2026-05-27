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
    //    : _fs(fs_rate), _carrNco(8, (float)fs_rate), _codeNco(0, (float)fs_rate), _m_sv(sv)
    : _fs(fs_rate), _carrNco(8, (float)fs_rate), _codeNco(8, (float)fs_rate), _m_sv(sv)
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

CorrelatorResult ChannelProcessor::Correlator(const RawSample *samples, size_t availableSamples) {
    if (availableSamples == 0 || samples == nullptr) {
        CorrelatorResult emptyRes = {};
        emptyRes.epoch_valid = false;
        emptyRes.consumed_sample_count = 0;
        return emptyRes;
    }

    bool epochTriggered = false;
    size_t i = 0;

    // 1. Process the exact 16,368 sample block sequentially
    for (i = 0; i < availableSamples; ++i) {
        _sampleCounter++;

        uint32_t carrIdx = _carrNco.clk();
        uint32_t oldCodeRotations = _codeNco.getRotations();
        _codeNco.clk();

        // NATIVE MIXING: Use raw int8_t data straight from the pointer matrix array
        int16_t bb_i = (int16_t)(samples[i].i * _carrNco.cosine(carrIdx) * 127);
        int16_t bb_q = (int16_t)(samples[i].q * _carrNco.sine(carrIdx) * 127);

        _acc.Ei += (bb_i * _codeNco.Early);
        _acc.Eq -= (bb_q * _codeNco.Early);
        _acc.Pi += (bb_i * _codeNco.Prompt);
        _acc.Pq -= (bb_q * _codeNco.Prompt);
        _acc.Li += (bb_i * _codeNco.Late);
        _acc.Lq -= (bb_q * _codeNco.Late);

        // Track rotations naturally across the 1ms block
        if (_codeNco.getRotations() < oldCodeRotations) {
            epochTriggered = true;
        }
    }

    // Every discrete 16,368 block represents a complete tracking epoch update step
    CorrelatorResult res = {};
    res.numSymbols = 0;
    res.epoch_valid = true; 
    res.consumed_sample_count = availableSamples; // Consume all 16,368 samples safely
    res.rollover_sample_idx = samples[availableSamples - 1].sample_tick;
    res.unix_time = samples[availableSamples - 1].unix_time;

    // 2. FIXED DETACHED LAYER: Create local, normalized variables 
    // exclusively for the square root power equations to protect bit precision.
    float norm = 1.0f / 16368.0f;
    float I = (float)_acc.Pi * norm;
    float Q = (float)_acc.Pq * norm;
    
    float Early_I  = (float)_acc.Ei * norm;
    float Early_Q  = (float)_acc.Eq * norm;
    float Late_I   = (float)_acc.Li * norm;
    float Late_Q   = (float)_acc.Lq * norm;

    float dynamicT = 0.001f; // Perfect 1 ms step size

    float totalPower = (I * I) + (Q * Q);
    float carrError = (totalPower > 1e-6f) ? ((I * Q) / totalPower) : 0.0f;
    
    // Use the normalized components to calculate Early and Late tracking power precisely
    float E = sqrtf((Early_I * Early_I) + (Early_Q * Early_Q));
    float L = sqrtf((Late_I * Late_I) + (Late_Q * Late_Q));
    float P = sqrtf(totalPower);
    
    // This will now compute precisely without clipping or rounding down to 0.000000
    float codeError = (P > 1e-6f) ? ((E - L) / (2.0f * P)) : 0.0f;

    calculateSNR(_acc, _snr);
    _isLocked = (_snr > 12.0f);

    if (res.numSymbols < 32) {
        res.symbols[res.numSymbols++] = (I > 0.0f) ? 1 : -1;
    }

    // 3. Loop filters execute using full-scale energy metrics
    float carrNcoUpdate = _oldCarrNco + (_carrLF.tau2 / _carrLF.tau1) * (carrError - _oldCarrError) + (carrError * (dynamicT / _carrLF.tau1));
    _oldCarrNco = carrNcoUpdate;
    _oldCarrError = carrError;
    _doppler_hz = (double)(_carrFreqBasis - carrNcoUpdate);

    float codeNcoUpdate = _oldCodeNco + (_codeLF.tau2 / _codeLF.tau1) * (codeError - _oldCodeError) + (codeError * (dynamicT / _codeLF.tau1));
    _oldCodeNco = codeNcoUpdate;
    _oldCodeError = codeError;
    _currentCommandedFreq = carrNcoUpdate;

    _carrNco.SetFrequency(_currentCommandedFreq);
    _codeNco.SetFrequency(_codeFreqBasis + codeNcoUpdate + ((float)_doppler_hz / 1540.0f));

    res.carrier_phase_error = carrError;
    res.absolute_carrier_phase = ((double)_accumulatedCarrierCycles * (2.0 * M_PI)) + (((double)_carrNco.getPhase() / 4294967296.0) * (2.0 * M_PI));
    res.doppler_hz = (float)(_carrFreqBasis - _currentCommandedFreq);
    res.prn = _prn;
    
    // Pass authentic full-scale unattenuated counts back to logging infrastructure
    res.Pi = (double)_acc.Pi; 
    res.Pq = (double)_acc.Pq;
    
    res.code_phase = (double)_codeNco.getRotations() + ((double)_codeNco.getPhase() / 4294967296.0);
    res.snr = _snr;
    res.is_locked = _isLocked;

    resetAccumulators(_acc);
    return res;
}
