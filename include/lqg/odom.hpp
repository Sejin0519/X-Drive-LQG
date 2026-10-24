#pragma once
#include "util.hpp"
#include "pros/rotation.hpp"
#include "pros/imu.hpp"
#include "pros/rtos.hpp"
#include "config.hpp"
#include "Eigen/Dense"
#include <optional>


class Odometry {
public:
    explicit Odometry(const Config& config);

    /**
     * @brief Reset sensors and begin odometry task.
     * 
     */
    void initialize();

    /**
     * @brief Set the new pose of the robot.
     * 
     * @param newPose the new pose to set. Theta should be in radians.
     */
    void setPose(Pose newPose);

    /**
     * @brief Set the Pose of the robot using x, y, and theta values.
     * 
     * @param x The new x position of the robot.
     * @param y The new y position of the robot.
     * @param theta The new orientation of the robot in radians.
     */
    void setPose(double x, double y, double theta);

    /**
     * @brief Get the Pose of the robot.
     * 
     * @param radians As default, return in degrees. If true, return in radians.
     * @return Pose of the robot.
     */
    Pose getPose(bool radians = true); //rad->ccw, deg->cw

    /**
     * @brief Get the state of the robot in vector form.
     * 
     * @return state of the robot [x, y, theta].
     */
    Eigen::Vector3d getState(); //x, y, theta

private:
    Pose pose;
    Config config;
    Pose prevPose;

    pros::Rotation verticalEncoder;
    pros::Rotation horizontalEncoder;
    pros::Imu imu;

    pros::Mutex mutex;
    std::optional<pros::Task> odomTask = std::nullopt;

    double prevHorizontalDist, prevVerticalDist;
    double prevTheta;

    double saveTime;
    void update();
    Eigen::Vector3d getSensorReadings();
};