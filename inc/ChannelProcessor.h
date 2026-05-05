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
    double i_val;
    double q_val;
    double current_code_phase;
};

class ChannelProcessor
{
public:
    // Default constructor so 'chan' can exist before acquisition
    ChannelProcessor() : _fs(16368000.0), _nco(10, 16368000.0f), 
        _carrNco(5, 16368000.0f), _codeNco(5, 16368000.0f), _code_phase(0) {}
    // The real constructor we use after lock
    ChannelProcessor(double fs_rate, const AcqResult &init);
    CorrRes process(const uint8_t *data, size_t count);
    //double getCodePhase() const { return _code_phase; }
    //double getLiveCode() {return this->_code_phase; }
    int getPRN() const {return _prn;}

private:
    NCO _nco;       // Keep for PCS / Acquisition logic
    NCO _carrNco; // Carrier NCO (Initial ~4.092 MHz)
    NCO _codeNco; // Code NCO (Initial ~1.023 MHz)
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