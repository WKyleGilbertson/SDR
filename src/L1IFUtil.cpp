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

TimeTrio get_timeData(uint32_t unixSeconds, uint32_t sampleTick, uint32_t Fs) {
    TimeTrio tme3;
    uint16_t samples_per_ms = (uint16_t)(Fs / 1000);
    uint8_t packets_per_ms = (uint8_t)(samples_per_ms / 2046);
    uint16_t total_packets_in_second = (uint16_t)(Fs / 2046); // IF alway 1023 pkt
    uint16_t current_packet_in_second = (sampleTick % Fs) / 2046;
        tme3.msCount = (uint16_t)(current_packet_in_second / packets_per_ms);
        tme3.fracMS  = (uint8_t)(current_packet_in_second % packets_per_ms);
        tme3.unixSecond = unixSeconds;
    return tme3;
};