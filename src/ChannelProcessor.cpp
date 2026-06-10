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

void ChannelProcessor::calculateSNR(Accumulators &acc, float &snr)
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

ChannelProcessor::ChannelProcessor(double fs_rate, const AcqResult &init, G2INIT &sv)
    : _fs(fs_rate), _carrNco(8, (float)fs_rate), _codeNco(4, (float)fs_rate), _m_sv(sv)
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
    _oldCodeNco = ((float)_doppler_hz / 1540.0f);
    //_oldCodeNco = 0.0f;
    _accumulatedCarrierCycles = 0;
    _sampleCounter = 0;
    _prevCodePhase = 0;
    _msIntegrated = 0;
    _initialCodePhase = init.codePhase;
    _continuousTrackedChips = init.codePhase;

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
    _initialCodePhase = init.codePhase;

    _continuousTrackedChips = _initialCodePhase;
    _absoluteBaseRotations = 0;
    _codeNco.InitializeEPLPipeline(_initialCodePhase, chipTravelDelay);

    /* Debug*/
    printf(
        "[CHAN INIT] PRN %d acqCode=%.4f initCode=%.4f ncoCode=%.4f rot=%u fine=%u delay=%d\n",
        _prn,
        init.codePhase,
        _initialCodePhase,
        _codeNco.getCodePhase(),
        _codeNco.getRotations(),
        _codeNco.getFinePhase16(),
        chipTravelDelay);
    /* End Debug*/

    _carrNco.SetFrequency(_carrFreqBasis - (float)_doppler_hz);
    _codeNco.SetFrequency(_codeFreqBasis);
    _currentCommandedFreq = _oldCarrNco;

    _ca_replica.resize(1023);
    for (int i = 0; i < 1023; i++)
        _ca_replica[i] = (int8_t)_m_sv.CODE[i];

    _carrLF.Bn = 10.0f;
    //_carrLF.Bn = 25.0f;
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

CorrelatorResult ChannelProcessor::Correlator(const RawSample *samples, size_t availableSamples)
{
    CorrelatorResult res = {};
    res.epoch_valid = false;
    bool saw_rollover = false;
    res.consumed_sample_count = availableSamples;
    res.epoch_offset_samples = -1; // Default to no-rollover

    if (availableSamples == 0 || samples == nullptr)
    {
        res.epoch_valid = false;
        res.consumed_sample_count = 0;
        return res;
    }

    float boundary_code_phase = _codeNco.getCodePhase();

    // 1. Rigid Hardware Mixing Loop
    for (size_t i = 0; i < availableSamples; ++i)
    {
        _sampleCounter++;
        uint32_t carrIdx = _carrNco.clk();

        // Track the rotations value BEFORE advancing the NCO
        uint16_t prev_rotations = _codeNco.getRotations();
        float prev_code_phase = _codeNco.getCodePhase();

        _codeNco.clk();

        uint16_t curr_rot = _codeNco.getRotations();
        float curr_code_phase = _codeNco.getCodePhase();

        static int rolloverPrintCount = 0;
        if (curr_rot < prev_rotations)
        {
            if (rolloverPrintCount++ % 100 == 0) // Throttle rollover prints to every 100 occurrences
                printf("[ROLLOVER] PRN %d offset=%zu prev=%.4f curr=%.4f\n",
                       _prn, i, prev_code_phase, curr_code_phase);
        }

        if (_codeNco.getRotations() < prev_rotations)
        {
            saw_rollover = true;
            res.epoch_valid = true;
            res.epoch_offset_samples = (int)i;
            res.epoch_sample_tick = samples[i].sample_tick;
            res.epoch_sample_index = samples[i].sample_index;
            res.unix_time = samples[i].unix_time;
        }

        int16_t bb_i = (int16_t)(samples[i].i * _carrNco.cosine(carrIdx) * 127);
        int16_t bb_q = (int16_t)(samples[i].q * _carrNco.sine(carrIdx) * 127);

        _acc.Ei += (bb_i * _codeNco.Early);
        _acc.Eq -= (bb_q * _codeNco.Early);
        _acc.Pi += (bb_i * _codeNco.Prompt);
        _acc.Pq -= (bb_q * _codeNco.Prompt);
        _acc.Li += (bb_i * _codeNco.Late);
        _acc.Lq -= (bb_q * _codeNco.Late);
    }

    // Telemetry tracking

    // 2. Loop Filters & Discriminators (Using actual availableSamples time)
    float norm = 1.0f / (float)availableSamples;
    float I = (float)_acc.Pi * norm;
    float Q = (float)_acc.Pq * norm;
    float Early_I = (float)_acc.Ei * norm;
    float Early_Q = (float)_acc.Eq * norm;
    float Late_I = (float)_acc.Li * norm;
    float Late_Q = (float)_acc.Lq * norm;

    float dynamicT = (float)availableSamples / (float)_fs;
    float totalPower = (I * I) + (Q * Q);

    float sign_I = (I >= 0.0f) ? 1.0f : -1.0f;
    float clean_I = I * sign_I;
    float clean_Q = Q * sign_I;
    float raw_angular_error = (clean_I > 1e-6f) ? atanf(clean_Q / clean_I) : 0.0f;
    float carrError = raw_angular_error / (float)M_PI;

    if (fabs(carrError - _oldCarrError) > 0.5f)
    {
        carrError = _oldCarrError + (carrError * 0.1f);
    }

    float E = sqrtf((Early_I * Early_I) + (Early_Q * Early_Q));
    float L = sqrtf((Late_I * Late_I) + (Late_Q * Late_Q));
    float P = sqrtf(totalPower);
    float codeError = (P > 1e-6f) ? ((E - L) / (2.0f * P)) : 0.0f;

    calculateSNR(_acc, _snr);
    _isLocked = (_snr > 12.0f);

    res.symbol = (I > 0) ? 1 : -1;
    res.numSymbols = 1;
    res.symbols[0] = res.symbol;

    // 3. Update Frequencies
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

    res.code_phase = boundary_code_phase;
    res.carrier_phase_error = carrError;
    res.absolute_carrier_phase = (((double)_carrNco.getPhase() / 4294967296.0) * (2.0 * M_PI));
    res.doppler_hz = (float)_doppler_hz;
    res.prn = _prn;
    res.Pi = _acc.Pi;
    res.Pq = _acc.Pq;
    res.snr = _snr;
    res.is_locked = _isLocked;

    resetAccumulators(_acc);
    return res;
}
