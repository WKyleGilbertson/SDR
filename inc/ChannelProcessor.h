#pragma once
#include <cstdint>
#include <cstddef>
#include <vector>
#include "NCO.h"
#include "g2init.h"
#include "L1IFUtil.hpp"  // Has the bit unpacking
#include "PCSEngine.hpp" // This defines AcqResult

struct CorrRes
{
    double Pi;
    double Pq;
    double code_phase;
    std::vector<int8_t> symbols; // hold the sign of acc_Pi for each ms
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
    //double getCodePhase() const { return _code_phase; }
    //double getLiveCode() {return this->_code_phase; }
    int getPRN() const {return _prn;}

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
    // Filter Coefficients
    float _tau1Carr, _tau2Carr, _tau1Code, _tau2Code;
    double _fs;
    int _prn;
    double _doppler_hz;
    double _code_phase;
    std::vector<int8_t> _ca_replica;
};

struct ChannelState {
    int prn;
    AcqResult result;   // Metadata from PCS (PRN, CodePhase, Bin)
    G2INIT    sv;       // The Gold Code replica (bits)
    std::unique_ptr<ChannelProcessor> processor; // The active tracker
    double handoverPhase = 0.0;
    bool isSynchronized = false;
    bool isActive() const {return processor != nullptr;}
    ChannelState() : prn(0), sv(0, 0) {}
};