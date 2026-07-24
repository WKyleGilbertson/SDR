#pragma once

#include "Ephemeris.hpp" // Ensure this matches the exact filename of your Ephemeris struct

// A simple 3D vector struct to hold Earth-Centered, Earth-Fixed (ECEF) coordinates
struct Vector3 {
    double x;
    double y;
    double z;
};

class PVTSolver {
public:
    // ========================================================================
    // WGS-84 CONSTANTS
    // These define the exact size, shape, and gravity of the Earth for GPS math
    // ========================================================================
    static constexpr double WGS84_MU = 3.986005e14;          // Earth's gravitational parameter (m^3/s^2)
    static constexpr double WGS84_OMEGA_E = 7.2921151467e-5; // Earth's rotation rate (rad/sec)
    static constexpr double SPEED_OF_LIGHT = 299792458.0;    // Speed of light in a vacuum (m/s)

    // ========================================================================
    // PUBLIC SOLVER METHODS
    // ========================================================================

    /**
     * Calculates the Satellite's exact X, Y, Z coordinates in space.
     * @param eph The decoded ephemeris struct for the satellite.
     * @param transmitTime The exact GPS Time of Week (TOW) the signal was transmitted.
     * @return A Vector3 containing the ECEF coordinates in meters.
     */
    static Vector3 calculateSatPosition(const Ephemeris& eph, double transmitTime);

    // --- FUTURE METHODS WE WILL IMPLEMENT NEXT ---
    // static double calculatePseudorange(double transmitTime, double receiveTime);
    // static PositionSolution computeUserPosition(const std::vector<Vector3>& satPositions, 
    //                                             const std::vector<double>& pseudoranges);
};