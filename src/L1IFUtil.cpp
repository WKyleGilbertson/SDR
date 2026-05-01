// L1IFUtil.cpp - Implementation of utility functions for L1 Interface
#include "L1IFUtil.hpp"

std::string get_iso8601_timestamp(uint32_t unix_time, uint16_t ms_offset) {
    time_t seconds = (time_t)unix_time;
    struct tm *timeinfo = std::gmtime(&seconds);
    char buffer[64];
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
//    uint16_t total_packets_in_second = (uint16_t)(Fs / 2046); // IF alway 1023 pkt
    uint16_t current_packet_in_second = (sampleTick % Fs) / 2046;
        tme3.msCount = (uint16_t)(current_packet_in_second / packets_per_ms);
        tme3.fracMS  = (uint8_t)(current_packet_in_second % packets_per_ms);
        tme3.unixSecond = unixSeconds;
    return tme3;
};

static int16_t map_bits(uint8_t m, uint8_t s) {
    int16_t val = (s == 0) ? 1 : -1;
    if (m != 0) val *= 3;
    return val << 3; // Shifts values to +/- 8 and +/- 24 
}

struct LUTContainer {
    UnpackEntry fnhn[256];
    UnpackEntry fnln[256];

    LUTContainer() {
        for (int b = 0; b < 256; ++b) {
            // FNHN Population
            fnhn[b].s0.i = map_bits((b >> 4) & 1, (b >> 5) & 1);
            fnhn[b].s0.q = map_bits((b >> 6) & 1, (b >> 7) & 1);
            fnhn[b].s1.i = map_bits((b >> 0) & 1, (b >> 1) & 1);
            fnhn[b].s1.q = map_bits((b >> 2) & 1, (b >> 3) & 1);

            // FNLN Population
            fnln[b].s0.i = map_bits((b >> 0) & 1, (b >> 1) & 1);
            fnln[b].s0.q = map_bits((b >> 2) & 1, (b >> 3) & 1);
            fnln[b].s1.i = map_bits((b >> 4) & 1, (b >> 5) & 1);
            fnln[b].s1.q = map_bits((b >> 6) & 1, (b >> 7) & 1);
        }
    }
};

// This static instance is created in writable memory (.data)
static LUTContainer& GetContainer() {
    static LUTContainer instance;
    return instance;
}

const UnpackEntry* GetLUT_FNHN() { return GetContainer().fnhn; }
const UnpackEntry* GetLUT_FNLN() { return GetContainer().fnln; }