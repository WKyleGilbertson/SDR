#pragma once
#include <cstdint>
#include <cstddef>
#include <vector>
#include <cstdlib>
#include <cstdio>
#include <deque>
#include "NCO.h"
#include "g2init.h"
#include "L1IFUtil.hpp"  // Has the bit unpacking
#include "PCSEngine.hpp" // This defines AcqResult

//#define SAMPLE_TRACE
//static constexpr float DEBUG_CODE_PHASE_SHIM =  0.0f;

struct EpochResult
{
    int32_t Ei;
    int32_t Eq;
    int32_t Pi;
    int32_t Pq;
    int32_t Li;
    int32_t Lq;
    uint32_t sample_count;
    uint64_t sample_index;
    uint32_t sample_tick;
    uint32_t unix_time;
    int offset_samples;
    int8_t symbol;
};
struct CorrelatorResult
{
    int prn = 0;
    int32_t Ei = 0;
    int32_t Eq = 0;
    int32_t Pi = 0;
    int32_t Pq = 0;
    int32_t Li = 0;
    int32_t Lq = 0;
    float E_mag = 0.0f;
    float P_mag = 0.0f;
    float L_mag = 0.0f;
    float code_error = 0.0f;
    float carrier_phase_error = 0.0f;    // Relative error: atanf(Pq / Pi)
    float absolute_carrier_phase = 0.0f; // Total continuous accumulated radians <--- ADD THIS
    float code_phase = 0.0f;
    float doppler_hz = 0.0f;
    float snr = 0.0f;
    float carrier_nco_hz = 0.0f;
    float code_nco_hz = 0.0f;
    uint64_t epoch_sample_index = 0;
    uint32_t epoch_sample_tick = 0;
    uint32_t unix_time = 0;
    bool is_locked = false;
    bool epoch_valid = false;
    int8_t symbol = 0;
    int8_t symbols[32] = {0};
    int numSymbols = 0;
    size_t consumed_sample_count = 0; // Number of samples consumed from the input buffer for this epoch's processing
    int epoch_offset_samples = -1;
    size_t epoch_sample_count = 0;
    std::vector<EpochResult> epochs;
};
struct LoopFilter
{
    float tau1;
    float tau2;
    float gain;
    float Bn;
    float zeta;
    float omega_n;
};

enum class LoopMode
{
    Acquisition,
    PullIn,
    Tracking
};

struct Accumulators
{
    int32_t Ei, Eq, Pi, Pq, Li, Lq, SEi, SEq, SLi, SLq;
};

struct TrackingMetrics
{
    float I = 0.0f;
    float Q = 0.0f;

    float Early_I = 0.0f;
    float Early_Q = 0.0f;
    float Prompt_I = 0.0f;
    float Prompt_Q = 0.0f;
    float Late_I = 0.0f;
    float Late_Q = 0.0f;

    float E = 0.0f;
    float P = 0.0f;
    float L = 0.0f;

    float E2 = 0.0f;
    float P2 = 0.0f;
    float L2 = 0.0f;

    float carrError = 0.0f;
    float codeError = 0.0f;

    float dynamicT = 0.001f;
};

class ChannelProcessor
{
public:
    // Default constructor so 'chan' can exist before acquisition
    ChannelProcessor() : _fs(ReceiverConfig::DEF_SAMPLE_RATE),
                         _carrNco(8, ReceiverConfig::DEF_SAMPLE_RATE),
                         _codeNco(0, ReceiverConfig::DEF_SAMPLE_RATE),
                         _code_phase(0), _m_sv(0, 0), _sampleCounter(0) {}
    // The real constructor we use after lock
    ChannelProcessor(double fs_rate, const AcqResult &init, G2INIT &sv,
         bool verboseInit = true);
    ~ChannelProcessor();
    CorrelatorResult Correlator(const RawSample *samples, size_t availableSamples);
    int getPRN() const { return _prn; }
    bool isLocked() const { return _isLocked; }
    float getSNR() const { return (float)_snr; }
    void setInputIsComplex(bool is_complex) { _input_is_complex = is_complex; }
    void setLoopEnables(bool enable_pll, bool enable_dll);
//    void setSampleDump(FILE * pf, int max_samples);
    void enableSampleTrace( const char *filename, size_t samples);
    void setSampleGain(float gain) {_sampleGain = gain;};
    void dumpSampleTrace(
        const RawSample &sample,
        uint32_t carrIdx,
        float c,
        float s,
        float bb_i,
        float bb_q,
        int32_t prompt_i_term,
        int32_t prompt_q_term);
    void setVerboseInit(bool enabled) {_verboseInit = enabled;}

private:
    void runAccumulation(
        const RawSample *samples,
        size_t availableSamples,
        CorrelatorResult &res);

    void harvestEpochResult(CorrelatorResult &res, const RawSample &sample,
                            size_t offset_samples);

    TrackingMetrics computeEpochDiscriminators(
        const Accumulators& acc, size_t sampleCount);

    void updateCarrierLoop(
        const TrackingMetrics &m);

    void updateCodeLoop(
        const TrackingMetrics &m);

    void fillResult(
        CorrelatorResult &res,
        const TrackingMetrics &m,
        float boundary_code_phase);

    NCO _carrNco; // Carrier NCO (Initial ~4.092 MHz)
    NCO _codeNco; // Code NCO (Initial ~1.023 MHz)
    G2INIT _m_sv;
    size_t _samplesPerMs;
    uint8_t _msIntegrated = 0;
    uint16_t _epochSampleCounter = 0;
    uint64_t _absoluteBaseRotations = 0;
    //    float _sampleFractionAccumulator = 0.0;
    float _continuousTrackedChips = 0.0;
    // Loop Filter State from TrkBST.cpp
    float _carrFreqBasis;
    float _codeFreqBasis;
    float _oldCodeError = 0.0f, _oldCodeNco = 0.0f;
    float _oldCarrError = 0.0f, _oldCarrNco = 0.0f;
    float _currentCommandedFreq = 0.0f;
    Accumulators _acc;
    Accumulators _epochAcc;
    uint32_t _epochSampleCount = 0;
    uint64_t _sampleCounter = 0;
    float _sampleGain = 4.0f;
    void resetAccumulators(Accumulators &acc);
    void calculateSNR(Accumulators &acc, float &snr);
    void setLoopMode(LoopMode mode);
    void calculateLoopCoefficients( LoopFilter &lf,
        float Bn   = -1.0f, float zeta = -1.0f, float gain = -1.0f);
    // Filter Coefficients
    LoopFilter _codeLF, _carrLF;
    uint32_t _trackingEpochs = 0;
    uint32_t _pllHoldoffEpochs = 0;
    uint32_t _pllGuardTrips = 0;
    float _fs;
    int _prn;
    float _initialDopplerHz = 0.0f;
    float _doppler_hz;
    float _code_phase;
    float _initialCodePhase;
    float _snr;
    bool _isLocked;
    uint32_t _prevCodePhase = 0;
    std::vector<int8_t> _ca_replica;
    int64_t _accumulatedCarrierCycles = 0; // Tracks the total number of carrier integer overflows
    float _snrBufferI[20];                 // 20 ms tracking window
    float _snrBufferQ[20];                 // 20 ms tracking window
    int _snrBufferIndex = 0;
    bool _input_is_complex = true;
    bool _enable_pll = true;
    bool _enable_dll = true;
    bool _fixed_nco_debug = false;
    FILE * _sampleDump = nullptr;
    int _sampleDumpRemaining = 0;
    bool _verboseInit = true;
};