#pragma once
#include "config.hpp"
#include "pros/motors.hpp"
#include "kinematics.hpp"
#include "Eigen/Dense"
#include "util.hpp"

class Chassis {
public:
    /**
     * @brief Constructor for the Chassis class.
     * @param config The configuration for the chassis.
     */
    explicit Chassis(const Config& config);

    /**
     * @brief Stop the chassis.
     */
    void stop();

    /**
     * @brief Compute and apply the motor voltage based on the desired acceleartion and velocity in the local frame
     * @note Computed voltages over the 12V range are saturated and clamped to prevent exceeding the maximum voltage.

     * @param accel acceleration vector.
     * @param vel velocity vector. Used to determine static friction and back EMF.
     * @return The resulting forces after motor voltage saturation.
     */
    Eigen::Vector3d applyLocalWrench(const Eigen::Vector3d& accel, const Eigen::Vector3d& vel);

    /**
     * @brief Get the average speeds of all motors.
     * @note Doesn't account for external gear ratio and wheel diameter.
     * @return The wheel speeds.
     */
    Eigen::Vector4d wheelSpeeds() const;

    /**
     * @brief Move the chassis with a specified voltage.
     * @param voltage The voltage to apply.
     */
    void moveVoltage(double voltage);

    /**
     * @brief Turn the chassis with a specified voltage.
     * @param voltage The voltage to apply.
     */
    void turnVoltage(double voltage);

    /**
     * @brief Get the average velocity of the wheels.
     * @return The average wheel velocity.
     */
    double avgWheelVel();

    /**
     * @brief Get the velocity of the turning wheels.
     * @return The turning wheel velocity.
     */
    double turnWheelVel();
private:
    Config config;
    std::array<pros::Motor, 4> motors; //[BR, FR, FL, BL]
};