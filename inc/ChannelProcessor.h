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

/*
struct RawSample
{
    int8_t i;             // Raw unscaled I value directly from MAX2769
    int8_t q;             // Raw unscaled Q value directly from MAX2769
    uint32_t sample_tick; // Inherited directly from meta.sample_tick
    uint32_t unix_time;
}; */

struct CorrelatorResult
{
    int prn;
    double Pi;
    double Pq;
    double carrier_phase_error;    // Relative error: atanf(Pq / Pi)
    double absolute_carrier_phase; // Total continuous accumulated radians <--- ADD THIS
    double code_phase;
    double doppler_hz;
    double snr;
    uint64_t rollover_sample_idx;
    uint32_t unix_time;
    bool is_locked;
    bool epoch_valid;
    int8_t symbols[32];
    int numSymbols;
    size_t consumed_sample_count; // Number of samples consumed from the input buffer for this epoch's processing
    int rollover_sample_index_in_block;
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
    double _sampleFractionAccumulator = 0.0;
    double _continuousTrackedChips = 0.0;
    // Loop Filter State from TrkBST.cpp
    float _carrFreqBasis;
    float _codeFreqBasis;
    float _oldCodeError = 0.0f, _oldCodeNco = 0.0f;
    float _oldCarrError = 0.0f, _oldCarrNco = 0.0f;
    float _currentCommandedFreq = 0.0f;
    Accumulators _acc;
    uint64_t _sampleCounter = 0;
    void resetAccumulators(Accumulators &acc);
    void ChannelProcessor::calculateSNR(Accumulators &acc, double &snr);
    // Filter Coefficients
    LoopFilter _codeLF, _carrLF;
    double _fs;
    int _prn;
    double _doppler_hz;
    double _code_phase;
    double _initialCodePhase;
    double _snr;
    bool _isLocked;
    uint32_t _prevCodePhase = 0;
    std::vector<int8_t> _ca_replica;
    int64_t _accumulatedCarrierCycles = 0; // Tracks the total number of carrier integer overflows
    float _snrBufferI[20];                 // 20 ms tracking window
    float _snrBufferQ[20];                 // 20 ms tracking window
    int _snrBufferIndex = 0;
};