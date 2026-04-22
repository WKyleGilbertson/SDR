#pragma once
#include <cstdint>
#include <cstddef>
#include "NCO.h"

// Define this so the compiler knows what 'process' returns
struct CorrRes {
    double i_val;
    double q_val;
};

class ChannelProcessor {
public:
    // Adding a default constructor to fix the SDR_test.cpp(14) error
    ChannelProcessor() : _fs(16368000.0), _nco(10, 16368000.0f) {}
    ChannelProcessor(double fs_rate);

    // Return the struct we just defined
    CorrRes process(const uint8_t* data, size_t count, double freq);

private:
    NCO _nco;
    double _fs;
    inline double _map(int sign, int mag) {
        return (sign ? -1.0 : 1.0) * (mag ? 3.0 : 1.0);
    }
};