#include "chassis.hpp"
#include <cmath>

Chassis::Chassis(const Config& config)
    : config(config),
      motors{ pros::Motor(config.motorPorts[0]),   // BR
              pros::Motor(config.motorPorts[1]),   // FR
              pros::Motor(config.motorPorts[2]),   // FL
              pros::Motor(config.motorPorts[3]) }  // BL
{}

Eigen::Vector4d Chassis::wheelSpeeds() const{ //rad/s
    Eigen::Vector4d wheelSpeeds;
    for(int i = 0; i < 4; i++) {
        wheelSpeeds(i) = motors[i].get_actual_velocity() * M_PI / 30.0; //rpm to rad/s
    }
    return wheelSpeeds;
}

void Chassis::applyLocalForce(const Eigen::Vector3d& localForce) {
    Eigen::Vector4d wheelForces = localForcesToWheelForces(localForce, config.centerToWheel);
    Eigen::Vector4d currentWheelSpeeds = wheelSpeeds();
    for(int i = 0; i < 4; i++) {
        motors[i].move_voltage(wheelForceToVoltage(wheelForces(i), currentWheelSpeeds(i), config));
    }
}
