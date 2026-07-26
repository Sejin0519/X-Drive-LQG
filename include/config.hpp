#pragma once
#include <array>
#include <cstdint>

struct Config {
    //chassis geometry
    double centerToWheel;
    double wheelRadius;
    //dynamics
    double mass;
    double inertia;

    std::int8_t motorPorts[4];

    //odometry
    double verticalEncoderPort, horizontalEncoderPort, imuPort;
    double horizontalOffset, verticalOffset;
    double horizontalDiam, verticalDiam;

    //control
    double dt; //discretizing interval

    //inverse dynamics
    double kF, kV, vStatic, vMax;

    double qPos, qVel, rTrans;
    double qAng, qAngVel, rRot;
};

