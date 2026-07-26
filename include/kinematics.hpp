#pragma once

#include "Eigen/Dense"
#include <cmath>

Eigen::Vector4d localForcesToWheelForces(const Eigen::Vector3d& localForces, double centerToWheelDistance);