#pragma once
#include "config.hpp"
#include "pros/motors.hpp"
#include "dynamics.hpp"
#include "kinematics.hpp"
#include "Eigen/Dense"

class Chassis {
public:
    explicit Chassis(const Config& config);

    
    void stop();
    void applyLocalForce(const Eigen::Vector3d& localForce);
    Eigen::Vector4d wheelSpeeds() const;

private:
    Config config;
    std::array<pros::Motor, 4> motors;
};