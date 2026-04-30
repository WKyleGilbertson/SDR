// L1IFUtil.cpp - Implementation of utility functions for L1 Interface
#include "L1IFUtil.hpp"

std::string get_iso8601_timestamp(uint32_t unix_time, uint32_t ms_offset) {
    time_t seconds = (time_t)unix_time;
    struct tm *timeinfo = std::gmtime(&seconds);

    char buffer[64]; // Correctly declared as a character array (buffer)
    
    // Format the date/time part. strftime returns 0 if the buffer is too small.
    if (std::strftime(buffer, sizeof(buffer), "%Y-%m-%dT%H:%M:%S", timeinfo) == 0) {
        return "FORMAT_ERROR";
    }

    std::ostringstream oss;
    // Standard ISO 8601 includes milliseconds after a decimal point
    oss << buffer << "." << std::setfill('0') << std::setw(3) << ms_offset << "Z";
    return oss.str();
}
