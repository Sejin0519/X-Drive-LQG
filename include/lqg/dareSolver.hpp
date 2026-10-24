#pragma once

#include "Eigen/Dense"
#include <utility>

/**
 * @brief Solves the discrete-time algebraic riccati equation of a given 2x2 system.
 * 
 * @param A System Matrix
 * @param B Input Matrix
 * @param Q State Cost Matrix
 * @param R Input Cost Matrix
 * @param tolerance Convergence tolerance; if the change in solution is less than this value, the algorithm returns the current solution
 * @param maxIteration Maximum number of iterations; if the algorithm fails to converge within this range, it returns a failure flag
 * @return A pair containing the success status and the solution matrix K.
 */
std::pair<bool, Eigen::Matrix<double, 1, 2>> dareSolver(const Eigen::Matrix2d& A, const Eigen::Vector2d& B,
    const Eigen::Matrix2d& Q, double R, double tolerance = 1e-6, int maxIteration = 5000);