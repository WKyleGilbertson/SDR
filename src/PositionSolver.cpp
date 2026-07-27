#define _USE_MATH_DEFINES
#include "PositionSolver.hpp"
#include "PVTSolver.hpp" // For Vector3
#include <cmath>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include <iostream>


GeodeticCoordinates PositionSolver::ecefToLLA(const Vector3& ecef) {
//inline GeodeticCoordinates ecefToLLA(const Vector3& ecef) {
    // WGS-84 ellipsoid constants
    constexpr double a = 6378137.0;          // Semi-major axis in meters
    constexpr double f = 1.0 / 298.257223563;// Flattening
    constexpr double b = a * (1.0 - f);      // Semi-minor axis
    constexpr double e2 = f * (2.0 - f);     // First eccentricity squared
    constexpr double ep2 = (a*a - b*b)/(b*b);// Second eccentricity squared

    double x = ecef.x;
    double y = ecef.y;
    double z = ecef.z;

    double p = std::sqrt(x*x + y*y);
    double theta = std::atan2(z * a, p * b);

    double lon = std::atan2(y, x);
    double lat = std::atan2(z + ep2 * b * std::sin(theta) * std::sin(theta) * std::sin(theta),
                            p - e2 * a * std::cos(theta) * std::cos(theta) * std::cos(theta));

    double N = a / std::sqrt(1.0 - e2 * std::sin(lat) * std::sin(lat));
    double alt = p / std::cos(lat) - N;

    GeodeticCoordinates geo;
    geo.latitudeDegrees = lat * (180.0 / M_PI);
    geo.longitudeDegrees = lon * (180.0 / M_PI);
    geo.altitudeMeters = alt;
    return geo;
}

PositionSolution PositionSolver::computePosition(
    const std::vector<Vector3>& satPositions,
    const std::vector<double>& pseudoranges,
    const std::vector<float>& snrs)
{
    PositionSolution solution;
    solution.isValid = false;
    solution.gdop = 99.9;

    size_t numSats = satPositions.size();
    if (numSats < 4 || pseudoranges.size() != numSats || snrs.size() != numSats) {
        return solution;
    }

    Eigen::Vector4d state;
    state << -1280000.0, -4700000.0, 4000000.0, 0.0; // Seed

    Eigen::MatrixXd H(numSats, 4);
    Eigen::VectorXd deltaRho(numSats);
    Eigen::MatrixXd W = Eigen::MatrixXd::Zero(numSats, numSats); // Weight matrix

    int maxIterations = 10;
    for (int iter = 0; iter < maxIterations; ++iter) 
    {
        for (size_t i = 0; i < numSats; ++i) 
        {
            double dx = satPositions[i].x - state(0);
            double dy = satPositions[i].y - state(1);
            double dz = satPositions[i].z - state(2);
            double expectedRange = std::sqrt(dx*dx + dy*dy + dz*dz);
            
            deltaRho(i) = pseudoranges[i] - (expectedRange + state(3));

            H(i, 0) = -dx / expectedRange;
            H(i, 1) = -dy / expectedRange;
            H(i, 2) = -dz / expectedRange;
            H(i, 3) = 1.0;

            // --- BUILD WEIGHT MATRIX (W) ---
            // Convert SNR (dB) to a linear scale weight. 
            // Higher SNR = higher weight. (e.g., variance scaling or direct linear mapping)
            // Using a simple power-based or linear multiplier:
            float snrLinear = std::pow(10.0f, snrs[i] / 10.0f);
            W(i, i) = snrLinear; 
        }

        // --- WEIGHTED LEAST SQUARES MATRIX OPERATION ---
        // Formula: deltaState = (H^T * W * H)^-1 * H^T * W * deltaRho
        Eigen::Matrix4d J = H.transpose() * W * H;
        Eigen::Vector4d deltaState = J.inverse() * H.transpose() * W * deltaRho;

        state += deltaState;

        if (deltaState.head<3>().norm() < 1e-3) {
            solution.isValid = true;
            break;
        }
    }

    if (solution.isValid) 
    {
        solution.ecefPosition.x = state(0);
        solution.ecefPosition.y = state(1);
        solution.ecefPosition.z = state(2);
        solution.clockBiasSeconds = state(3) / PVTSolver::SPEED_OF_LIGHT;
        
        Eigen::Matrix4d covariance = (H.transpose() * W * H).inverse();
        solution.gdop = std::sqrt(covariance.trace());
    }

    return solution;
}