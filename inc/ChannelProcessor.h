#pragma once
#include <cstdint>
#include <cstddef>
#include <vector>
#include <cstdio>
#include <deque>
#include "NCO.h"
#include "g2init.h"
#include "L1IFUtil.hpp"  // Has the bit unpacking
#include "PCSEngine.hpp" // This defines AcqResult

struct CorrelatorResult
{
    int prn = 0;
    int32_t Pi = 0;
    int32_t Pq = 0;
    float carrier_phase_error = 0.0f;    // Relative error: atanf(Pq / Pi)
    float absolute_carrier_phase = 0.0f; // Total continuous accumulated radians <--- ADD THIS
    float code_phase = 0.0f;
    float doppler_hz = 0.0f;
    float snr = 0.0f;
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

struct Accumulators
{
    int32_t Ei, Eq, Pi, Pq, Li, Lq, SEi, SEq, SLi, SLq;
};

class ChannelProcessor
{
public:
    // Default constructor so 'chan' can exist before acquisition
    ChannelProcessor() : _fs(16368000.0), _carrNco(8, 16368000.0f),
                         _codeNco(0, 16368000.0f), _code_phase(0), _m_sv(0, 0),
                         _sampleCounter(0) {}
    // The real constructor we use after lock
    ChannelProcessor(double fs_rate, const AcqResult &init, G2INIT &sv);
    CorrelatorResult Correlator(const RawSample *samples, size_t availableSamples);
    int getPRN() const { return _prn; }
    bool isLocked() const { return _isLocked; }
    float getSNR() const { return (float)_snr; }

private:
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
    uint64_t _sampleCounter = 0;
    void resetAccumulators(Accumulators &acc);
    void ChannelProcessor::calculateSNR(Accumulators &acc, float &snr);
    // Filter Coefficients
    LoopFilter _codeLF, _carrLF;
    float _fs;
    int _prn;
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
};