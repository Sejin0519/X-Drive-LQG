#include "dynamics.hpp"
#include <cmath>
#include <algorithm>



double wheelForceToVoltage(double force, double wheelSpeed, const Config& config) {
    double volts = config.kF * force + config.kV * wheelSpeed + config.vStatic * ((force > 0) - (force < 0));
    volts = std::clamp(volts, -config.vMax, config.vMax);
    return volts * 1000.0; //V to mV for move_voltage()
}