#include "kinematics.hpp"

Eigen::Vector4d localForcesToWheelForces(const Eigen::Vector3d& localForces, double centerToWheelDistance) {
    Eigen::Matrix<double, 4, 3> inverseKinematics;
    Eigen::Vector4d wheelForces; //[BR, FR, FL, BL]

    double sqrt2 = std::sqrt(2);
    double R = 1.0 / centerToWheelDistance;
    inverseKinematics << 
    sqrt2, sqrt2, R, //BR
    sqrt2, -sqrt2, -R, //FR
    sqrt2, sqrt2, -R, //FL
    sqrt2, -sqrt2, R; //BL

    inverseKinematics = inverseKinematics / 4.0;

    wheelForces = inverseKinematics * localForces;
    return wheelForces;
}

Eigen::Vector3d wheelForcesToLocalForces(const Eigen::Vector4d& wheelForces, double centerToWheelDistance) {
    Eigen::Matrix<double, 3, 4> forwardKinematics;
    Eigen::Vector3d localForces; //[x, y, theta]

    double invSqrt2 = 1.0 / std::sqrt(2);
    double R = centerToWheelDistance;
    forwardKinematics << 
    invSqrt2, invSqrt2, invSqrt2, invSqrt2, //x
    invSqrt2, -invSqrt2, invSqrt2, -invSqrt2, //y
    R, -R, -R, R; //theta

    localForces = forwardKinematics * wheelForces;

    return localForces;
}

Eigen::Vector4d bodyMotionToWheelMotion(const Eigen::Vector3d& bodyMotion, double centerToWheelDistance, double wheelRadius, double gearRatio) {
    Eigen::Matrix<double, 4, 3> inverseKinematics;
    Eigen::Vector4d wheelMotion; //[BR, FR, FL, BL]

    double invSqrt2 = 1.0 / std::sqrt(2);

    inverseKinematics << 
    invSqrt2, invSqrt2, centerToWheelDistance, //BR
    invSqrt2, -invSqrt2, -centerToWheelDistance, //FR
    invSqrt2, invSqrt2, -centerToWheelDistance, //FL
    invSqrt2, -invSqrt2, centerToWheelDistance; //BL

    wheelMotion = inverseKinematics * bodyMotion * (gearRatio / wheelRadius);
    return wheelMotion;
}