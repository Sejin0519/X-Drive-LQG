#pragma once

#include "Eigen/Dense"
#include <cmath>


/**
 * @brief Distributes local forces to wheel forces based on X-drive kinematics.
 * 
 * @param localForces Local force vector [Fx, Fy, Ftheta]
 * @param centerToWheelDistance Distance from the center of the robot to the wheels
 * @return Force along each wheels direction
 */
Eigen::Vector4d localForcesToWheelForces(const Eigen::Vector3d& localForces, double centerToWheelDistance);

/**
 * @brief Compute the net body-frame force produced by four wheel forces.
 * 
 * @param wheelForces Force along each wheel's direction 
 * @param centerToWheelDistance Distance from the center of the robot to the wheels
 * @return body-frame force vector [Fx, Fy, Ftheta]
 */
Eigen::Vector3d wheelForcesToLocalForces(const Eigen::Vector4d& wheelForces, double centerToWheelDistance);

/**
 * @brief Converts body-frame velocity or acceleration to motor velocities or accelerations
 * 
 * @param bodyMotion body-frame velocity or acceleration vector
 * @param centerToWheelDistance Distance from the center of the robot to the wheels
 * @param wheelRadius Radius of the wheels
 * @param gearRatio External gear ratio between the motors and the wheels
 * @return Motor velocities or acceleration depending on bodyMotion input
 */
Eigen::Vector4d bodyMotionToWheelMotion(const Eigen::Vector3d& bodyMotion, double centerToWheelDistance, double wheelRadius, double gearRatio);