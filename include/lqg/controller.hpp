#pragma once
#include <array>
#include "config.hpp"
#include "chassis.hpp"
#include "dareSolver.hpp"
#include "model.hpp"
#include "odom.hpp"
#include "trajectory.hpp"

class Controller {
public:
    /**
     * @brief Constructor for the Controller class.
     * 
     * Computes the LQR and LQE gain matrices based on the config parameters and the double integrator model
     * Rumbles the controller if any of the DARE solutions failed to converge 
     * 
     * @param chassis The chassis to control.
     * @param odom The odometry for position tracking.
     * @param config The configuration for the controller.
     */
    Controller(Chassis& chassis, Odometry& odom, Config& config);

    /**
     * @brief Follows a trajectory using LQR control.
     * 
     * Updates the control input at each time step based on the current state and the reference trajectory
     * 
     * @pre runMotion() must be called before calling this function.
     * 
     * @param trajectory reference trajectory to follow. Contains a list of position, velocity and accerlation with timestamps
     * @param async If true, runs the trajectory in a separate task and returns immediately.
     */
    void followTrajectory(const trajectory::PathData& trajectory, bool async = true);

    /**
     * @brief Cancels the current motion.
     */
    void cancelMotion();

    /**
     * @brief Checks if the robot is currently following a trajectory.
     * 
     * @return true if the robot is in motion, false if it's not.
     */
    bool isInMotion() const {return motionRunning.load();}

    /**
     * @brief Runs the updateMotion function in a separate task.
     * 
     * Initializes the filtered state with the current odometry reading.
     */
    void runMotion();

    /**
     * @brief Waits until the current motion is complete.
     * 
     */
    void waitUntilDone();
private:

    Chassis& chassis;
    Odometry& odom;
    Config& config;

    std::optional<pros::Task> motionTask = std::nullopt;

    std::atomic<bool> cancelRequested{false};
    std::atomic<bool> motionRunning{false};

    pros::Mutex motionMutex;
    Vector6d xHat;
    Eigen::Vector3d uApplied = Eigen::Vector3d::Zero();

    // DEBUG: temporary filtered-state logging, remove after use
    static constexpr int kLogSize = 2000;
    std::array<Vector6d, kLogSize> debugLog;
    std::array<std::uint32_t, kLogSize> debugLogTime;
    int debugLogCount = 0;

    Eigen::Matrix<double, 3, 6> K;
    Eigen::Matrix<double, 6, 3> Kf;
    FullModel model;


    /**
     * @brief Runs an infinite loop that updates the filtered state based on odometry readings and applied control inputs.
     */
    void updateMotion();
};