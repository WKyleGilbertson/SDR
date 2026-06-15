// #define DEBUG_ROLLOVER
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
    _carrFreqBasis = 4.092e6f + init.bin * 500.0f;
    _codeFreqBasis = 1.023e6f;
    resetAccumulators(_acc);
    resetAccumulators(_epochAcc);
    _epochSampleCount = 0;
    _snr = -999.0f;
    _isLocked = false;
    _samplesPerMs = (size_t)(_fs / 1000.0);
    _prn = init.prn;
    _doppler_hz = init.bin * 500.0f;
    _oldCarrNco = 0.0f;
    _oldCarrError = 0.0f;
    //_oldCarrNco = _carrFreqBasis - (float)_doppler_hz;
    _oldCodeError = 0.0f;
    _oldCodeNco = ((float)_doppler_hz / 1540.0f);
    //_oldCodeNco = 0.0f;
    _accumulatedCarrierCycles = 0;
    _sampleCounter = 0;
    _prevCodePhase = 0;
    _msIntegrated = 0;
    /*
    _initialCodePhase = init.codePhase;
    */
   _initialCodePhase = init.codePhase + DEBUG_CODE_PHASE_SHIM;

while (_initialCodePhase < 0.0f)
    _initialCodePhase += 1023.0f;

while (_initialCodePhase >= 1023.0f)
    _initialCodePhase -= 1023.0f;

    for (int i = 0; i < 20; ++i)
    {
        _snrBufferI[i] = 0.0f;
        _snrBufferQ[i] = 0.0f;
    }
    _snrBufferIndex = 0;

    _codeNco.LoadCACODE(_m_sv.CACODE);
    _codeNco.RakeSpacing(CorrelatorSpacing::halfChip);

    float spc = (float)_fs / 1023000.0f;
     int chipTravelDelay = 0;
    //int chipTravelDelay = (int)std::round(32.0f / spc);

    _continuousTrackedChips = _initialCodePhase;
    _absoluteBaseRotations = 0;
    _codeNco.InitializeEPLPipeline(_initialCodePhase, chipTravelDelay);

    /* Debug*/
printf(
    "[CHAN INIT DETAIL] "
    "acq=%.4f nco=%.4f rot=%u fine=%u "
    "E=%d P=%d L=%d delay=%d\n",
    init.codePhase,
    _codeNco.getCodePhase(),
    _codeNco.getRotations(),
    _codeNco.getFinePhase16(),
    _codeNco.Early,
    _codeNco.Prompt,
    _codeNco.Late,
    chipTravelDelay);
    /* End Debug*/

    //_currentCommandedFreq = _oldCarrNco;
    _currentCommandedFreq = _carrFreqBasis;
    //_carrNco.SetFrequency(_carrFreqBasis - (float)_doppler_hz);
    _carrNco.SetFrequency(_currentCommandedFreq);
    _codeNco.SetFrequency(_codeFreqBasis);

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

void ChannelProcessor::runAccumulation(
    const RawSample *samples,
    size_t availableSamples,
    CorrelatorResult &res)
{
    for (size_t i = 0; i < availableSamples; ++i)
    {
        _sampleCounter++;

        uint32_t carrIdx = _carrNco.clk();
        uint16_t prev_rotations = _codeNco.getRotations();
        float prev_code_phase = _codeNco.getCodePhase();
        _codeNco.clk();

#ifdef DEBUG_ROLLOVER
        uint16_t curr_rot = _codeNco.getRotations();
        float curr_code_phase = _codeNco.getCodePhase();
        static int rolloverPrintCount = 0;
        if (curr_rot < prev_rotations)
        {
            if (rolloverPrintCount++ % 100 == 0)
            {
                printf("[ROLLOVER] PRN %d offset=%zu prev=%.4f curr=%.4f\n",
                       _prn, i, prev_code_phase, curr_code_phase);
            }
        }
#endif

        int16_t s = (int16_t)(_carrNco.sine(carrIdx) * 127.0f);
        int16_t c = (int16_t)(_carrNco.cosine(carrIdx) * 127.0f);
        int16_t in_i = samples[i].i;
        int16_t in_q = samples[i].q;
        int16_t bb_i = 0;
        int16_t bb_q = 0;

        if (_input_is_complex)
        {
            // (I + jQ) * (cos - j sin)
            bb_i = (int16_t)(in_i * c + in_q * s);
            bb_q = (int16_t)(in_q * c - in_i * s);
        }
        else
        {
            // Real IF: sample * local oscillator
            bb_i = (int16_t)(in_i * c);
            bb_q = (int16_t)(-in_i * s);
        }

        _acc.Ei += (bb_i * _codeNco.Early);
        _acc.Eq += (bb_q * _codeNco.Early);
        _acc.Pi += (bb_i * _codeNco.Prompt);
        _acc.Pq += (bb_q * _codeNco.Prompt);
        _acc.Li += (bb_i * _codeNco.Late);
        _acc.Lq += (bb_q * _codeNco.Late);
        _epochAcc.Ei += (bb_i * _codeNco.Early);
        _epochAcc.Eq += (bb_q * _codeNco.Early);
        _epochAcc.Pi += (bb_i * _codeNco.Prompt);
        _epochAcc.Pq += (bb_q * _codeNco.Prompt);
        _epochAcc.Li += (bb_i * _codeNco.Late);
        _epochAcc.Lq += (bb_q * _codeNco.Late);

        _epochSampleCount++;

        if (_codeNco.getRotations() < prev_rotations)
        {
            harvestEpochResult(res, samples[i], i);
        }
    }
}

void ChannelProcessor::harvestEpochResult(
    CorrelatorResult &res, const RawSample &sample, size_t offset_samples)
{
    res.epoch_valid = true;
    res.epoch_offset_samples = (int)offset_samples;
    res.epoch_sample_tick = sample.sample_tick;
    res.epoch_sample_index = sample.sample_index;
    res.unix_time = sample.unix_time;

    EpochResult epoch = {};
    epoch.Ei = _epochAcc.Ei;
    epoch.Eq = _epochAcc.Eq;
    epoch.Pi = _epochAcc.Pi;
    epoch.Pq = _epochAcc.Pq;
    epoch.Li = _epochAcc.Li;
    epoch.Lq = _epochAcc.Lq;
    epoch.sample_count = _epochSampleCount;
    epoch.sample_index = sample.sample_index;
    epoch.sample_tick = sample.sample_tick;
    epoch.unix_time = sample.unix_time;
    epoch.offset_samples = (int)offset_samples;
    epoch.symbol = (_epochAcc.Pi >= 0) ? 1 : -1;

    res.epochs.push_back(epoch);

    resetAccumulators(_epochAcc);
    _epochSampleCount = 0;
}

TrackingMetrics ChannelProcessor::computeDiscriminators(
    size_t availableSamples)
{
    TrackingMetrics m = {};

    float norm = 1.0f / (float)availableSamples;

    m.I = (float)_acc.Pi * norm;
    m.Q = (float)_acc.Pq * norm;

    m.Early_I = (float)_acc.Ei * norm;
    m.Early_Q = (float)_acc.Eq * norm;
    m.Prompt_I = m.I;
    m.Prompt_Q = m.Q;
    m.Late_I = (float)_acc.Li * norm;
    m.Late_Q = (float)_acc.Lq * norm;
    m.dynamicT = (float)availableSamples / (float)_fs;
    m.P2 = m.I * m.I + m.Q * m.Q;
    float sign_I = (m.I >= 0.0f) ? 1.0f : -1.0f;
    float clean_I = m.I * sign_I;
    float clean_Q = m.Q * sign_I;
    float raw_angular_error =
        (clean_I > 1e-6f)
            ? atanf(clean_Q / clean_I)
            : 0.0f;

    m.carrError = raw_angular_error / (float)M_PI;

    if (fabs(m.carrError - _oldCarrError) > 0.5f)
    {
        m.carrError = _oldCarrError + (m.carrError * 0.1f);
    }

    m.E2 = m.Early_I * m.Early_I + m.Early_Q * m.Early_Q;
    m.L2 = m.Late_I * m.Late_I + m.Late_Q * m.Late_Q;
    m.E = sqrtf(m.E2);
    m.P = sqrtf(m.P2);
    m.L = sqrtf(m.L2);

    m.codeError =
        ((m.E2 + m.L2) > 1e-6f)
            ? ((m.E2 - m.L2) / (m.E2 + m.L2))
            : 0.0f;

    calculateSNR(_acc, _snr);
    _isLocked =
        (_snr > 12.0f);

    return m;
}

void ChannelProcessor::updateCarrierLoop(
    const TrackingMetrics &m)
{
    float carrNcoUpdate = _oldCarrNco + (_carrLF.tau2 / _carrLF.tau1) *
                                            (m.carrError - _oldCarrError) *
                                            (m.dynamicT / _carrLF.tau1);
    _oldCarrNco = carrNcoUpdate;
    _oldCarrError = m.carrError;
    _currentCommandedFreq = _carrFreqBasis - carrNcoUpdate;
    _carrNco.SetFrequency(_currentCommandedFreq);
    _doppler_hz = _currentCommandedFreq - 4.092e6f;
}

void ChannelProcessor::updateCodeLoop(
    const TrackingMetrics &m)
{
    float codeNcoUpdate = _oldCodeNco + (_codeLF.tau2 / _codeLF.tau1) *
                                            (m.codeError - _oldCodeError) *
                                            (m.dynamicT / _codeLF.tau1);
    _oldCodeNco = codeNcoUpdate;
    _oldCodeError = m.codeError;
    _codeNco.SetFrequency(_codeFreqBasis + codeNcoUpdate +
                          ((float)_doppler_hz / 1540.0f));
}

void ChannelProcessor::fillResult(
    CorrelatorResult &res,
    const TrackingMetrics &m,
    float boundary_code_phase)
{
    res.code_phase = boundary_code_phase;
    res.carrier_phase_error = m.carrError;
    res.absolute_carrier_phase = (float)(((double)_carrNco.getPhase() /
                                          4294967296.0) *
                                         (2.0 * M_PI));
    res.doppler_hz = (float)_doppler_hz;
    res.prn = _prn;
    res.Pi = _acc.Pi;
    res.Pq = _acc.Pq;
    res.snr = _snr;
    res.is_locked = _isLocked;
    res.E_mag = m.E;
    res.P_mag = m.P;
    res.L_mag = m.L;
    res.code_error = m.codeError;
}

CorrelatorResult ChannelProcessor::Correlator(const RawSample *samples, size_t availableSamples)
{
    CorrelatorResult res = {0};
    res.epoch_valid = false;
    res.consumed_sample_count = availableSamples;
    res.epoch_offset_samples = -1; // Default to no-rollover

    if (availableSamples == 0 || samples == nullptr)
    {
        res.epoch_valid = false;
        res.consumed_sample_count = 0;
        return res;
    }

    float boundary_code_phase = _codeNco.getCodePhase();

    // 1. Reduce signal to BaseBand
    runAccumulation(samples, availableSamples, res);
    // 2. Find Tracking Errors
    TrackingMetrics m = computeDiscriminators(availableSamples);

    res.symbol = (m.I > 0) ? 1 : -1;
    res.numSymbols = 1;
    res.symbols[0] = res.symbol;

    // 3. Update Frequencies
    static constexpr bool OPEN_LOOP_TEST = false;

    if (!OPEN_LOOP_TEST)
    {
        updateCarrierLoop(m);
        updateCodeLoop(m);
    }

    fillResult(res, m, boundary_code_phase);

    resetAccumulators(_acc);
    return res;
}
