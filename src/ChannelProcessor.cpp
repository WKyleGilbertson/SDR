#include "ChannelProcessor.h"
#include <cmath>

/*void ChannelProcessor::setSampleDump(FILE * fp, int max_samples) {
    _sampleDump = fp;
    _sampleDumpRemaining = max_samples;
} */

void ChannelProcessor::enableSampleTrace(
    const char *filename,
    size_t samples)
{
    _sampleDump = fopen(filename, "w");

    if (_sampleDump)
    {
        fprintf(_sampleDump,
                "sample,rot,code_phase,...\n");

        _sampleDumpRemaining = samples;
    }
}

void ChannelProcessor::dumpSampleTrace(
    const RawSample &sample,
    uint32_t carrIdx,
    float c,
    float s,
    float bb_i,
    float bb_q,
    int32_t prompt_i_term,
    int32_t prompt_q_term)
{
    if (!_sampleDump || _sampleDumpRemaining == 0)
        return;

    fprintf(_sampleDump,
            "%llu,%u,%.6f,%u,%u,%d,%d,%d,%u,%u,%.9f,%.9f,"
            "%d,%d,%.6f,%.6f,%d,%d,%d,%d,%d,%d\n",
            (unsigned long long)_sampleCounter,
            _codeNco.getRotations(),
            _codeNco.getCodePhase(),
            _codeNco.getRotations(),
            _codeNco.getFinePhase16(),
            _codeNco.Early,
            _codeNco.Prompt,
            _codeNco.Late,
            _carrNco.getPhase(),
            carrIdx,
            c,
            s,
            sample.i,
            sample.q,
            bb_i,
            bb_q,
            prompt_i_term,
            prompt_q_term,
            _epochAcc.Pi,
            _epochAcc.Pq,
            _epochAcc.Ei,
            _epochAcc.Li);

    _sampleDumpRemaining--;
}

void ChannelProcessor::setLoopEnables(bool enable_pll, bool enable_dll)
{
    _enable_pll = enable_pll;
    _enable_dll = enable_dll;
}

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

void ChannelProcessor::calculateLoopCoefficients(
    LoopFilter &lf, float Bn, float zeta, float gain)
{
    if (Bn > 0.0f)
        lf.Bn = Bn;
    if (zeta > 0.0f)
        lf.zeta = zeta;
    if (gain > 0.0f)
        lf.gain = gain;

    lf.omega_n = lf.Bn * 8.0f * lf.zeta /
                 (4.0f * lf.zeta * lf.zeta + 1.0f);

    lf.tau1 = lf.gain / (lf.omega_n * lf.omega_n);
    lf.tau2 = 2.0f * lf.zeta / lf.omega_n;
}

void ChannelProcessor::setLoopMode(LoopMode mode)
{
    switch (mode)
    {
    case LoopMode::Acquisition:
        calculateLoopCoefficients(_carrLF, 40.0f, 0.707f, 1.0f);
        calculateLoopCoefficients(_codeLF, 20.0f, 0.707f, 1.0f);
        if (_verboseInit)
            fprintf(stdout, "\n[LOOPS] Acquisition\n");
        break;

    case LoopMode::PullIn:
        calculateLoopCoefficients(_carrLF, 30.0f, 0.707f, 1.0f);
        calculateLoopCoefficients(_codeLF, 10.0f, 0.707f, 1.0f);
        fprintf(stdout, "\n[LOOPS] Pull-In\n");
        break;

    case LoopMode::Tracking:
        calculateLoopCoefficients(_carrLF, 20.0f, 0.707f, 1.0f);
        calculateLoopCoefficients(_codeLF, 5.0f, 0.707f, 1.0f);
        fprintf(stdout, "\n[LOOPS] Tracking");
        break;
    }
}

ChannelProcessor::ChannelProcessor(double fs_rate, const AcqResult &init,
                                   G2INIT &sv, bool verboseInit)
    : _fs(fs_rate), _carrNco(8, (float)fs_rate), _codeNco(4, (float)fs_rate),
      _m_sv(sv), _verboseInit(verboseInit)
{
    _carrFreqBasis = ReceiverConfig::L1_IF_HZ + init.bin * 500.0f;
    _codeFreqBasis = ReceiverConfig::CODE_FREQ_HZ;
    resetAccumulators(_acc);
    resetAccumulators(_epochAcc);
    resetAccumulators(_epochAcc);
    resetAccumulators(_prevEpochAcc);
    _use_fll = true;
    _epochSampleCount = 0;
    _snr = -999.0f;
    _isLocked = false;
    _samplesPerMs = (size_t)(_fs / 1000.0);
    _prn = init.prn;
    _initialDopplerHz = init.bin * 500.0f;
    _doppler_hz = _initialDopplerHz;
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

    _initialCodePhase = init.codePhase;

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
    //_codeNco.RakeSpacing(CorrelatorSpacing::Narrow);

    float spc = (float)_fs / 1023000.0f;
    int chipTravelDelay = 0;
    // int chipTravelDelay = (int)std::round(32.0f / spc);

    _continuousTrackedChips = _initialCodePhase;
    _absoluteBaseRotations = 0;
    _codeNco.InitializeEPLPipeline(_initialCodePhase, chipTravelDelay);

    /* Debug
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

    _currentCommandedFreq = _carrFreqBasis;
    _carrNco.SetFrequency(_currentCommandedFreq);
    _codeNco.SetFrequency(_codeFreqBasis);

    _ca_replica.resize(1023);
    for (int i = 0; i < 1023; i++)
        _ca_replica[i] = (int8_t)_m_sv.CODE[i];

    setLoopMode(LoopMode::Acquisition);
    // setLoopMode(LoopMode::PullIn);
    // setLoopMode(LoopMode::Tracking);

    if (_verboseInit)
    {
        printf("[CHAN INIT] PRN %3d code=%8.4f %4u_%.2u F:%.1f dF: %7.1f\n",
               init.prn, init.codePhase, _codeNco.getRotations(),
               _codeNco.getFinePhase16(), _currentCommandedFreq, _doppler_hz);
    }
}

ChannelProcessor::~ChannelProcessor()
{
    if (_sampleDump)
        fclose(_sampleDump);
}

void ChannelProcessor::runAccumulation(
    const RawSample *samples,
    size_t availableSamples,
    CorrelatorResult &res)
{
    int rotation_changes = 0;
    int sample_ticks = 0;
    uint16_t last_rot = _codeNco.getRotations();
    for (size_t i = 0; i < availableSamples; ++i)
    {
        _sampleCounter++;

        uint16_t prev_rotations = _codeNco.getRotations();
        float prev_code_phase = _codeNco.getCodePhase();

        uint32_t carrIdx = _carrNco.getPhase() >> (32 - 8);

        // 1. Convert NCO sine/cosine to 8-bit integer for MAC
        int8_t s = (int8_t)(_carrNco.sine(carrIdx) * 127.0f);
        int8_t c = (int8_t)(_carrNco.cosine(carrIdx) * 127.0f);
        int16_t in_i = samples[i].i;
        int16_t in_q = samples[i].q;
        int16_t bb_i = 0;
        int16_t bb_q = 0;

        if (_input_is_complex)
        {
            // (I + jQ) * (cos - j sin) -> integer MAC (Multiply-Accumulate)
             bb_i = (int16_t)(in_i * c + in_q * s);
             bb_q = (int16_t)(in_q * c - in_i * s);
        }
        else
        {
            // Real IF: sample * local oscillator
             bb_i = (int16_t)(in_i * c);
             //bb_q = (int16_t)(-in_i * s);
             bb_q = (int16_t)(in_i * s);
        }

        _epochAcc.Ei += (int32_t)((int32_t)bb_i * _codeNco.Early);
        _epochAcc.Eq += (int32_t)((int32_t)bb_q * _codeNco.Early);
        _epochAcc.Pi += (int32_t)((int32_t)bb_i * _codeNco.Prompt);
        _epochAcc.Pq += (int32_t)((int32_t)bb_q * _codeNco.Prompt);
        _epochAcc.Li += (int32_t)((int32_t)bb_i * _codeNco.Late);
        _epochAcc.Lq += (int32_t)((int32_t)bb_q * _codeNco.Late);

        _epochSampleCount++;

        _carrNco.clk();
        _codeNco.clk();

        sample_ticks++;

        uint16_t now_rot = _codeNco.getRotations();
        if (now_rot != last_rot)
        {
            rotation_changes++;
            last_rot = now_rot;
        }

#ifdef SAMPLE_TRACE
        dumpSampleTrace(samples[i], carrIdx, c, s, bb_i, bb_q, prompt_i_term, prompt_q_term);
#endif

        if (_codeNco.getRotations() < prev_rotations)
        {

            _epochAcc.Ei >>= 4;
            _epochAcc.Eq >>= 4;
            _epochAcc.Pi >>= 4;
            _epochAcc.Pq >>= 4;
            _epochAcc.Li >>= 4;
            _epochAcc.Lq >>= 4;

            TrackingMetrics m =
                computeEpochDiscriminators(_epochAcc, _epochSampleCount);

            float carrier_before = _currentCommandedFreq;
            float old_nco_before = _oldCarrNco;
            float old_err_before = _oldCarrError;

            static int pll_print_count = 0;

            if (_enable_pll)
                updateCarrierLoop(m);

            float carrier_after = _currentCommandedFreq;

            if (_enable_dll)
                updateCodeLoop(m);

            harvestEpochResult(res, samples[i], i);
            fillResult(res, m, _codeNco.getCodePhase());
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

    res.Ei = _epochAcc.Ei;
    res.Eq = _epochAcc.Eq;
    res.Pi = _epochAcc.Pi;
    res.Pq = _epochAcc.Pq;
    res.Li = _epochAcc.Li;
    res.Lq = _epochAcc.Lq;
    res.epoch_sample_count = _epochSampleCount;

    resetAccumulators(_epochAcc);
    _epochSampleCount = 0;
}

TrackingMetrics ChannelProcessor::computeEpochDiscriminators(
    const Accumulators &acc, size_t sampleCount)
{
    TrackingMetrics m = {};

    float norm = 1.0f / (float)sampleCount;

    m.I = (float)acc.Pi * norm;
    m.Q = (float)acc.Pq * norm;

    m.Early_I = (float)acc.Ei * norm;
    m.Early_Q = (float)acc.Eq * norm;
    m.Prompt_I = m.I;
    m.Prompt_Q = m.Q;
    m.Late_I = (float)acc.Li * norm;
    m.Late_Q = (float)acc.Lq * norm;
    m.dynamicT = (float)sampleCount / (float)_fs;
    m.P2 = m.I * m.I + m.Q * m.Q;

    // --- NEW: FLL Cross-Product Discriminator ---
    float dot = (float)(acc.Pi * _prevEpochAcc.Pi + acc.Pq * _prevEpochAcc.Pq);
    float cross = (float)(acc.Pq * _prevEpochAcc.Pi - acc.Pi * _prevEpochAcc.Pq);

    // Returns the phase shift over the last 1ms epoch in radians
    m.fllError = atan2f(cross, dot);

    // Store current accumulators for the next epoch's FLL calculation
    _prevEpochAcc = acc;
    // --------------------------------------------

    float raw =
        atan2f(m.Q, m.I); // Returns [-pi, pi] radians
    if (raw > (M_PI / 2))
        raw -= M_PI;
    else if (raw < (-M_PI / 2))
        raw += M_PI;

    m.carrError = raw;
    // m.carrError = raw / (2.0f * (float)M_PI);

    m.E2 = m.Early_I * m.Early_I + m.Early_Q * m.Early_Q;
    m.L2 = m.Late_I * m.Late_I + m.Late_Q * m.Late_Q;
    m.E = sqrtf(m.E2);
    m.P = sqrtf(m.P2);
    m.L = sqrtf(m.L2);

    m.codeError =
        ((m.E2 + m.L2) > 1e-6f)
            ? ((m.E2 - m.L2) / (m.E2 + m.L2))
            : 0.0f;

    calculateSNR((Accumulators &)acc, _snr);
    _isLocked =
        (_snr > 12.0f);

    return m;
}

void ChannelProcessor::updateCarrierLoop(const TrackingMetrics &m)
{
    float carrNcoUpdate = _oldCarrNco;

    if (_use_fll)
    {
        // FLL MODE: m.fllError is in Hz.
        // We use a simple 1st-order frequency integrator to pull the signal in.
        // A gain of 0.1 to 0.5 allows fast frequency pull-in without chaotic overshooting.
        float fll_gain = 0.25f;

        carrNcoUpdate = _oldCarrNco + (fll_gain * m.fllError);
    }
    else
    {
        // PLL MODE: m.carrError is in Radians.
        // Standard Costas phase tracking for data decoding.
        float errorDelta = m.carrError - _oldCarrError;

        // Prevent aggressive NCO snaps on cycle slips
        if (fabsf(errorDelta) > 1.0f)
        {
            errorDelta = 0.0f;
        }
        carrNcoUpdate = _oldCarrNco +
                        ((_carrLF.tau2 / _carrLF.tau1) * errorDelta) +
                        ((m.dynamicT / _carrLF.tau1) * m.carrError);
    }

    _oldCarrNco = carrNcoUpdate;
    _oldCarrError = m.carrError;

    // Apply the correction to the basis frequency
    _currentCommandedFreq = _carrFreqBasis - carrNcoUpdate;
    _carrNco.SetFrequency(_currentCommandedFreq);

    // Compute active Doppler shift
    _doppler_hz = _currentCommandedFreq - ReceiverConfig::L1_IF_HZ;
}

void ChannelProcessor::updateCodeLoop(
    const TrackingMetrics &m)
{
    /*
    // For a carrier-aided DLL, a 1st order loop (proportional only) is highly stable.
        // The carrier aiding provides the velocity tracking. The DLL only provides phase alignment.

        // Proportional gain based on tau1/tau2 from your loop config, or just a simple gain
        float kp = 0.5f;

        float codeCorrection = kp * m.codeError;

        _oldCodeError = m.codeError; // Kept for logging if needed

        // NCO Frequency = Base + CarrierAiding + CodePhaseCorrection
        _codeNco.SetFrequency(_codeFreqBasis + codeCorrection +
                              ((float)_doppler_hz / 1540.0f));  */

    // Correct representation of a 2nd order loop update:
    float codeNcoUpdate = _oldCodeNco + (_codeLF.tau2 / _codeLF.tau1) * (m.codeError - _oldCodeError) +
                          (m.dynamicT / _codeLF.tau1) * m.codeError;
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
    res.carrier_nco_hz = _currentCommandedFreq;
    res.code_nco_hz = _codeFreqBasis + _oldCodeNco + ((float)_doppler_hz / 1540.0f);
    res.prn = _prn;
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

    float sample_epoch_code_phase = _codeNco.getCodePhase();
    // TODO: res.code_phase should mean sample-epoch code phase at start of 1 ms buffer.
    // Do not overload this with DLL discriminator or rollover residual.
    // 1. Reduce signal to BaseBand
    runAccumulation(samples, availableSamples, res);
    // 2. Find Tracking Errors Moved to runAccmulation

    res.code_phase = sample_epoch_code_phase;

    res.symbol = (res.Pi > 0) ? 1 : -1;
    res.numSymbols = 1;
    res.symbols[0] = res.symbol;
    // 3. Update Frequencies Moved to runAccumulation
    // 4. FillResult() moved to runAccumulation
    return res;
}
