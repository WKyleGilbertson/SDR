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
    const std::vector<double>& pseudoranges)
{
    PositionSolution solution;
    solution.isValid = false;
    solution.gdop = 99.9; // Default to poor geometry

    size_t numSats = satPositions.size();
    
    // We strictly need 4 satellites: X, Y, Z, and Time (Clock Bias)
    if (numSats < 4 || pseudoranges.size() != numSats) {
        return solution; 
    }

    // State vector: [X, Y, Z, c*dt]
    // We start our guess at the center of the Earth (0,0,0) with 0 clock bias.
    Eigen::Vector4d state = Eigen::Vector4d::Zero();

    Eigen::MatrixXd H(numSats, 4);     // Geometry Matrix (Jacobian)
    Eigen::VectorXd deltaRho(numSats); // Pseudorange residuals (Errors)

    int maxIterations = 10;
    
    for (int iter = 0; iter < maxIterations; ++iter) 
    {
        for (size_t i = 0; i < numSats; ++i) 
        {
            // Delta from current estimated position to the satellite
            double dx = satPositions[i].x - state(0);
            double dy = satPositions[i].y - state(1);
            double dz = satPositions[i].z - state(2);

            // Expected distance to the satellite from our current guess
            double expectedRange = std::sqrt(dx*dx + dy*dy + dz*dz);

            // Residual = Measured Range - (Expected Range + Receiver Clock Bias)
            deltaRho(i) = pseudoranges[i] - (expectedRange + state(3));

            // Populate the Jacobian Matrix 'H' (Direction Cosines)
            H(i, 0) = -dx / expectedRange;
            H(i, 1) = -dy / expectedRange;
            H(i, 2) = -dz / expectedRange;
            H(i, 3) = 1.0; // The clock bias affects all satellites equally
        }

        // --- THE MAGIC EIGEN MATRIX SOLVER ---
        // Equation: deltaState = (H^T * H)^-1 * H^T * deltaRho
        Eigen::Vector4d deltaState = (H.transpose() * H).inverse() * H.transpose() * deltaRho;

        // Apply the calculated correction to our current guess
        state += deltaState;

        // Convergence Check: If the correction moved us less than 1 millimeter, we have arrived!
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
        
        // Convert the distance bias back into seconds
        solution.clockBiasSeconds = state(3) / PVTSolver::SPEED_OF_LIGHT;
        
        // Calculate GDOP (Geometric Dilution of Precision)
        // This tells us how "good" our satellite geometry is. Lower is better.
        Eigen::Matrix4d covariance = (H.transpose() * H).inverse();
        solution.gdop = std::sqrt(covariance.trace());
    }

    return solution;
}