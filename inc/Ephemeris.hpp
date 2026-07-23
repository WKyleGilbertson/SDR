#pragma once
#include <cstdint>

struct Ephemeris {
    int prn;
    bool isValid = false;

    // --- Subframe 1: Clock Data ---
    uint32_t weekNumber;
    uint32_t toc;    // Time of Clock
    double   af0;    // Clock Bias
    double   af1;    // Clock Drift
    double   af2;    // Clock Drift Rate

    // --- Subframe 2: Orbit Data Part 1 ---
    uint32_t toe;    // Time of Ephemeris
    double   ecc;    // Eccentricity
    double   sqrta;  // Square root of semi-major axis
    double   m0;     // Mean anomaly at reference time
    double   crs;
    double   cuc;
    double   dn;
    double   cus;

    // --- Subframe 3: Orbit Data Part 2 ---
    double   i0;     // Inclination angle at reference time
    double   omega0; // Longitude of ascending node
    double   omega;  // Argument of perigee
    double   omegaDot; // Rate of right ascension
    double   iDot;   // Rate of inclination angle
    double   cic;
    double   cis;
    double   crc;
};