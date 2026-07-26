#include "kinematics.hpp"

Eigen::Vector4d localForcesToWheelForces(const Eigen::Vector3d& localForces, double centerToWheelDistance) {
    Eigen::Matrix<double, 4, 3> inverseKinematics;
    Eigen::Vector4d wheelForces; //[BR, FR, FL, BL]

    // Eigen::Matrix3d rotationMatrix;
    // rotationMatrix << std::cos(theta), std::sin(theta), 0,
    // -std::sin(theta), std::cos(theta), 0,
    // 0, 0, 1;

    // Eigen::Vector3d localForces = rotationMatrix * globalForces;

    double sqrt2 = std::sqrt(2);
    double R = 1.0 / centerToWheelDistance;
    inverseKinematics << sqrt2, sqrt2, R,
    sqrt2, -sqrt2, -R,
    sqrt2, sqrt2, -R,
    sqrt2, -sqrt2, R;

    inverseKinematics = inverseKinematics / 4.0;

    wheelForces = inverseKinematics * localForces;
    return wheelForces;
}

