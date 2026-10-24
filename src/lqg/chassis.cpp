#include "chassis.hpp"
#include <cmath>

Chassis::Chassis(const Config& config)
    : config(config),
      motors{ pros::Motor(config.motorPorts[0], pros::MotorGears::blue), //BR
              pros::Motor(config.motorPorts[1], pros::MotorGears::blue), //FR
              pros::Motor(config.motorPorts[2], pros::MotorGears::blue), //FL
              pros::Motor(config.motorPorts[3], pros::MotorGears::blue) } //BL
{
    for (pros::Motor& motor : motors) { //chassis owns the motors, so it sets the brake mode to brake
        motor.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    }
}

Eigen::Vector4d Chassis::wheelSpeeds() const{ //rad/s (more like motor shaft speed, not wheel speed)
    Eigen::Vector4d wheelSpeeds;
    for(int i = 0; i < 4; i++) {
        wheelSpeeds(i) = motors[i].get_actual_velocity() * M_PI / 30.0; //motor rpm to rad/s
    }
    return wheelSpeeds;
}

Eigen::Vector3d Chassis::applyLocalWrench(const Eigen::Vector3d& accel, const Eigen::Vector3d& vel) {
    //total speed and direction of the wheels, used for feedforward voltage calculation
    Eigen::Vector4d wheelVel = bodyMotionToWheelMotion(vel, config.centerToWheel, config.wheelRadius, config.gearRatio);

    //velocity and acceleration for translational and rotational components
    Eigen::Vector4d transVel = bodyMotionToWheelMotion({vel(0), vel(1), 0.0}, config.centerToWheel, config.wheelRadius, config.gearRatio);
    Eigen::Vector4d rotVel = bodyMotionToWheelMotion({0.0, 0.0, vel(2)}, config.centerToWheel, config.wheelRadius, config.gearRatio);

    Eigen::Vector4d transAccel = bodyMotionToWheelMotion({accel(0), accel(1), 0.0}, config.centerToWheel, config.wheelRadius, config.gearRatio);
    Eigen::Vector4d rotAccel = bodyMotionToWheelMotion({0.0, 0.0, accel(2)}, config.centerToWheel, config.wheelRadius, config.gearRatio);

    Eigen::Vector4d velV, accelV; 
    double s = 1.0; //clamping term
    for(int i = 0; i < 4; i++) {
        //motor model
        velV(i) = config.vStatic * sign(wheelVel(i)) + config.kV   * transVel(i) + config.kV_r * rotVel(i);
        accelV(i) = config.kA * transAccel(i) + config.kA_r * rotAccel(i);

        //clamp the acceleration to avoid exceeding max voltage
        if(accelV(i) < -1e-6) {
            s = std::min(s, (-config.vMax - velV(i)) / accelV(i));
        } else if(accelV(i) > 1e-6) {
            s = std::min(s, (config.vMax - velV(i)) / accelV(i));
        }
    }
    
    s = std::clamp(s, 0.0, 1.0);

    for(int i = 0; i < 4; i++) {
        motors[i].move_voltage((velV(i) + s * accelV(i)) * 1000.0);
    }

    return s * accel;
}

void Chassis::moveVoltage(double voltage) {
    for(int i = 0; i < 4; i++) {
        motors[i].move_voltage(voltage);
    }
}

void Chassis::turnVoltage(double voltage) {
    motors[0].move_voltage(voltage);
    motors[1].move_voltage(-voltage);
    motors[2].move_voltage(-voltage);
    motors[3].move_voltage(voltage);
}

double Chassis::avgWheelVel() {
    Eigen::Vector4d speeds = wheelSpeeds();
    return (speeds(0) + speeds(1) + speeds(2) + speeds(3)) / 4.0;
}

double Chassis::turnWheelVel() {
    Eigen::Vector4d speeds = wheelSpeeds();
    return (speeds(0) - speeds(1) - speeds(2) + speeds(3)) / 4.0;  
}

void Chassis::stop() {
    motors[0].move_voltage(0);
    motors[1].move_voltage(0);
    motors[2].move_voltage(0);
    motors[3].move_voltage(0);
}