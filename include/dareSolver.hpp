#pragma once

#include "Eigen/Dense"
#include <utility>

std::pair<bool, Eigen::Matrix<double, 1, 2>> dareSolver(const Eigen::Matrix2d& A, const Eigen::Vector2d& B, 
    const Eigen::Matrix2d& Q, double R, double tolerance = 1e-6, int maxIteration = 1000);