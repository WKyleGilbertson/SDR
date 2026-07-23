#pragma once
#include <unordered_map>
#include <mutex>
#include "Ephemeris.hpp"

class ConstellationManager {
public:
    static ConstellationManager& getInstance() {
        static ConstellationManager instance;
        return instance;
    }

    void commitEphemeris(int prn, const Ephemeris& ephem) {
        std::lock_guard<std::mutex> lock(_mtx);
        _ephemerisDB[prn] = ephem;
    }

    bool hasValidEphemeris(int prn) {
        std::lock_guard<std::mutex> lock(_mtx);
        return _ephemerisDB.count(prn) && _ephemerisDB[prn].isValid;
    }

    Ephemeris getEphemeris(int prn) {
        std::lock_guard<std::mutex> lock(_mtx);
        return _ephemerisDB[prn]; // Will return empty struct if not found
    }

private:
    ConstellationManager() = default;
    std::unordered_map<int, Ephemeris> _ephemerisDB;
    std::mutex _mtx;
};