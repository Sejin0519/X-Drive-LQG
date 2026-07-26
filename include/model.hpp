#pragma once
#include "Eigen/Dense"

struct SubsystemModel {
    Eigen::Matrix2d A;
    Eigen::Vector2d B;
};

SubsystemModel translationalModel(double mass, double dt);
SubsystemModel rotationalModel(double inertia, double dt);