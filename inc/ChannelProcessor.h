#pragma once
#include <cstdint>
#include <cstddef>
#include <vector>
#include "NCO.h"
#include "g2init.h"
#include "L1IFUtil.hpp"  // Has the bit unpacking
#include "NavDecoder.h"
#include "PCSEngine.hpp" // This defines AcqResult

struct CorrRes
{
    double Pi;
    double Pq;
    double code_phase;
    double dopplerHZ;
    double snr;
    std::vector<int8_t> symbols; 
};

struct ChannelTelemetry {
    int prn;
    double snr;
    double dopplerHz;
    double codePhase;
    bool isLocked;
};

struct LoopFilter {
    float tau1;
    float tau2;
    float gain;
    float Bn;
    float zeta;
    float omega_n;
};

struct Accumulators {
    int32_t Ei, Eq, Pi, Pq, Li, Lq, SEi, SEq, SLi, SLq;
};

class ChannelProcessor
{
public:
    // Default constructor so 'chan' can exist before acquisition
    ChannelProcessor() : _fs(16368000.0), _carrNco(10, 16368000.0f),
        _codeNco(0, 16368000.0f), _code_phase(0), _m_sv(0, 0) {}
    // The real constructor we use after lock
    ChannelProcessor(double fs_rate, const AcqResult &init, G2INIT sv);
    CorrRes process(const uint8_t *data, size_t count);
    int getPRN() const {return _prn;}
    bool isLocked() const {return _isLocked;}
    float getSNR() const {return (float)_snr;}
    ChannelTelemetry getTelemetry() const {
        return {_prn, _snr, _doppler_hz, _code_phase, _isLocked };
    }

private:
    NCO _carrNco; // Carrier NCO (Initial ~4.092 MHz)
    NCO _codeNco; // Code NCO (Initial ~1.023 MHz)
    G2INIT _m_sv;
    size_t _samplesPerMs;
    // Loop Filter State from TrkBST.cpp
    float _carrFreqBasis;
    float _codeFreqBasis;
    float _oldCodeError = 0.0f, _oldCodeNco = 0.0f;
    float _oldCarrError = 0.0f, _oldCarrNco = 0.0f;
    float _currentCommandedFreq = 0.0f;
    Accumulators _acc;
    void resetAccumulators(Accumulators & acc); 
    void ChannelProcessor::calculateSNR(Accumulators & acc, double & snr);
    int _rolloverDelayCounter = -1;
    // Filter Coefficients
    LoopFilter _codeLF, _carrLF;
    double _fs;
    int _prn;
    double _doppler_hz;
    double _code_phase;
    double _snr;
    bool _isLocked;
    std::vector<int8_t> _ca_replica;
    BitSync _sync = {}; // Initialize to zero
    int8_t _lastSymbol = 0; // To detect sign flips
    float _bitAccI = 0.0f;     // For the 20 ms coherent integration
    float _bitAccQ = 0.0f;     // For the 20 ms cohereent integration
};

class NavDecoder;
struct ChannelState {
    int prn;
    AcqResult result;   // Metadata from PCS (PRN, CodePhase, Bin)
    G2INIT    sv;       // The Gold Code replica (bits)
    std::unique_ptr<ChannelProcessor> processor; // The active tracker
    std::unique_ptr<NavDecoder> decoder;

    bool isActive() const {return processor != nullptr;}

    ChannelState(int p, double fs, const AcqResult& res, G2INIT s) 
        : prn(p), result(res), sv(s) {
        processor = std::make_unique<ChannelProcessor>(fs, result, s);
        decoder = std::make_unique<NavDecoder>(p);
    }

  // Since unique_ptr can't be copied, we need to allow the vector to MOVE it
    ChannelState(ChannelState&&) noexcept = default;
    ChannelState& operator=(ChannelState&&) noexcept = default;
    
    // Delete the default copy constructor
    ChannelState(const ChannelState&) = delete;
    ChannelState& operator=(const ChannelState&) = delete;  
};