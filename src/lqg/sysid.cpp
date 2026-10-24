#include "sysid.hpp"


SysID::SysID(Chassis& chassis)
    : chassis(chassis) {}

void SysID::moveRamp(double dV, double time) {
    double volts = 0.0;
    std::uint32_t startTime = pros::millis();
    while (pros::millis() - startTime < time) {
        chassis.moveVoltage(volts);
        printf("%d,%.1f,%.2f\n", pros::millis(), volts, chassis.avgWheelVel());
        volts += dV;
        pros::delay(10);
    }
    chassis.stop();
}

void SysID::moveStep(double targetMV, double time) {
    std::uint32_t startTime = pros::millis();
    while (pros::millis() - startTime < time) {
        chassis.moveVoltage(targetMV);
        printf("%d,%.1f,%.2f\n", pros::millis(), targetMV, chassis.avgWheelVel());
        pros::delay(10);
    }
    chassis.stop();
    pros::delay(500);
}

void SysID::turnRamp(double dV, double time) {
    double volts = 0.0;
    std::uint32_t startTime = pros::millis();
    while (pros::millis() - startTime < time) {
        chassis.turnVoltage(volts);
        printf("%d,%.1f,%.2f\n", pros::millis(), volts, chassis.turnWheelVel());
        volts += dV;
        pros::delay(10);
    }
    chassis.stop();
}

void SysID::turnStep(double targetMV, double time) {
    std::uint32_t startTime = pros::millis();
    while (pros::millis() - startTime < time) {
        chassis.turnVoltage(targetMV);
        printf("%d,%.1f,%.2f\n", pros::millis(), targetMV, chassis.turnWheelVel());
        pros::delay(10);
    }
    chassis.stop();
    pros::delay(500);

}
