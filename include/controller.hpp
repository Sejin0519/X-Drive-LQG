#pragma once
#include "config.hpp"
#include "chassis.hpp"
#include "dareSolver.hpp"
#include "model.hpp"
#include "odom.hpp"
#include "trajectory.hpp"

class Controller {
public:
    Controller(Chassis& chassis, Odometry& odom, Config& config);
    void followTrajectory(Trajectory& traj, bool async = true);
    void holdPose(const Pose& target);
    void cancelMotion();
    bool isInMotion() const;
    void initialize();
private:

    Chassis& chassis;
    Odometry& odom;
    Config& config;
    Eigen::RowVector2d kTrans, kRot;
    std::atomic<bool> cancelRequested{false};
    pros::Mutex motionMutex;

    Eigen::Matrix<double, 3, 6> K;
};