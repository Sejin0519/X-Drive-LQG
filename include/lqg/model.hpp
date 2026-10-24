#pragma once
#include "Eigen/Dense"

struct SubsystemModel {
    Eigen::Matrix2d A;
    Eigen::Vector2d B;
    Eigen::RowVector2d C{1.0, 0.0};
};

struct FullModel {
    Eigen::Matrix<double, 6, 6> A;
    Eigen::Matrix<double, 6, 3> B;
    Eigen::Matrix<double, 3, 6> C;
};

/**
 * @brief Creates a 2D double integrator model with a given time step.
 * [x, xdot]
 *
 * @param dt Time step for discretization
 * @return SubsystemModel  
 */
SubsystemModel doubleIntegrator(double dt);

/**
 * @brief Creates a full 6D model of the robot's dynamics with a given time step.
 * [x, y, theta, xdot, ydot, thetadot]
 *
 * @param dt Time step for discretization
 * @return FullModel 
 */
FullModel fullModel(double dt);