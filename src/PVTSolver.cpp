#include "PVTSolver.hpp"
#include <cmath>

Vector3 PVTSolver::calculateSatPosition(const Ephemeris& eph, double transmitTime) {
    
    // 1. Calculate time from ephemeris reference epoch (t_k)
    double tk = transmitTime - eph.toe;
    
    // Account for week crossovers (if tk is > half a week, subtract a week)
    if (tk >  302400.0) tk -= 604800.0;
    if (tk < -302400.0) tk += 604800.0;

    // 2. Compute Mean Anomaly (M_k)
    double A = eph.sqrta * eph.sqrta;
    double n0 = std::sqrt(WGS84_MU / (A * A * A)); // Computed mean motion
    double n = n0 + eph.dn;                        // Corrected mean motion
    double Mk = eph.m0 + n * tk;                   // Mean anomaly

    // 3. Solve Kepler's Equation for Eccentric Anomaly (E_k) iteratively
    double Ek = Mk;
    double Ek_old = 0.0;
    int iterations = 0;
    while (std::abs(Ek - Ek_old) > 1e-12 && iterations < 15) {
        Ek_old = Ek;
        Ek = Mk + eph.ecc * std::sin(Ek);
        iterations++;
    }

    // 4. Calculate True Anomaly (v_k)
    double sin_vk = (std::sqrt(1.0 - eph.ecc * eph.ecc) * std::sin(Ek)) / (1.0 - eph.ecc * std::cos(Ek));
    double cos_vk = (std::cos(Ek) - eph.ecc) / (1.0 - eph.ecc * std::cos(Ek));
    double vk = std::atan2(sin_vk, cos_vk);

    // 5. Calculate Argument of Latitude (Phi_k)
    double Phi_k = vk + eph.omega;

    // 6. Calculate Harmonic Corrections (Due to Earth's oblateness)
    double sin_2Phi = std::sin(2.0 * Phi_k);
    double cos_2Phi = std::cos(2.0 * Phi_k);
    
    double du_k = eph.cus * sin_2Phi + eph.cuc * cos_2Phi; // Argument of Latitude correction
    double dr_k = eph.crs * sin_2Phi + eph.crc * cos_2Phi; // Radius correction
    double di_k = eph.cis * sin_2Phi + eph.cic * cos_2Phi; // Inclination correction

    // 7. Apply Corrections
    double uk = Phi_k + du_k;                              // Corrected argument of latitude
    double rk = A * (1.0 - eph.ecc * std::cos(Ek)) + dr_k; // Corrected radius
    double ik = eph.i0 + di_k + eph.iDot * tk;             // Corrected inclination

    // 8. Positions in the Orbital Plane
    double x_prime = rk * std::cos(uk);
    double y_prime = rk * std::sin(uk);

    // 9. Corrected Longitude of Ascending Node (Omega_k)
    // This accounts for the Earth rotating underneath the satellite while the signal is flying
    double Omega_k = eph.omega0 + (eph.omegaDot - WGS84_OMEGA_E) * tk - WGS84_OMEGA_E * eph.toe;

    // 10. Rotate into Earth-Centered, Earth-Fixed (ECEF) Coordinates
    Vector3 pos;
    pos.x = x_prime * std::cos(Omega_k) - y_prime * std::cos(ik) * std::sin(Omega_k);
    pos.y = x_prime * std::sin(Omega_k) + y_prime * std::cos(ik) * std::cos(Omega_k);
    pos.z = y_prime * std::sin(ik);

    return pos;
}