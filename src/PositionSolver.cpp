#include "PositionSolver.hpp"
#include <cmath>
#include <iostream>

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