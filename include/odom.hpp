#pragma once
#include "util.hpp"
#include "pros/rotation.hpp"
#include "pros/imu.hpp"
#include "pros/rtos.hpp"
#include "config.hpp"
#include "Eigen/Dense"
#include <optional>
#include "types.hpp"


class Odometry {
public:
    explicit Odometry(const Config& config);

    void initialize();
    void setPose(Pose newPose);
    void setPose(double x, double y, double theta);
    Pose getPose(bool radians = true); //rad->ccw, deg->cw
    Vector6d getState(); //x, y, theta, xdot, ydot, thetadot

private:    
    Pose pose;
    Config config;
    Pose prevPose;
    Pose velocity;

    pros::Rotation verticalEncoder;
    pros::Rotation horizontalEncoder;
    pros::Imu imu;
    pros::Mutex mutex;
    std::optional<pros::Task> odomTask = std::nullopt;
    
    double prevHorizontalDist, prevVerticalDist;
    double prevTheta;

    double saveTime;
    void updateTask();
    void update();
    Eigen::Vector3d getSensorReadings();
};