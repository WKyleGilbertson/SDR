#pragma once

#include <vector>
#include <Eigen/Dense>
#include "PVTSolver.hpp" // For Vector3

struct PositionSolution {
    Vector3 ecefPosition;     // User X, Y, Z in meters
    double  clockBiasSeconds; // Receiver clock error
    double  gdop;             // Geometric Dilution of Precision (Accuracy metric)
    bool    isValid;
};

class PositionSolver {
public:
    /**
     * Solves for user position using Iterative Least Squares.
     * Requires a minimum of 4 satellites.
     * * @param satPositions ECEF coordinates of the satellites (Meters)
     * @param pseudoranges Measured pseudoranges to those satellites (Meters)
     * @return PositionSolution containing the calculated position and clock bias
     */
    static PositionSolution computePosition(
        const std::vector<Vector3>& satPositions,
        const std::vector<double>& pseudoranges);
};