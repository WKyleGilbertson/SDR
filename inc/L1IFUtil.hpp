// L1IFUtil.hpp - Utility functions for L1 Interface
#pragma once
#include <iostream>
#include <ctime>
#include <iomanip>
#include <sstream>
#include <string>
#include <cstdint>
struct TimeTrio {
    uint32_t unixSecond;
    uint16_t msCount;
    uint8_t  fracMS;
};

std::string get_iso8601_timestamp(uint32_t unix_time, uint32_t ms_offset);

TimeTrio get_timeData(uint32_t unixSeconds, uint32_t sampleTick, uint32_t Fs);