#pragma once
#include <array>
#include <cstdint>

constexpr double INCH = 0.0254;



struct Config {
    //chassis geometry
    double centerToWheel;
    double wheelRadius;
    double gearRatio;

    std::int8_t motorPorts[4];

    //odometry
    std::int8_t verticalEncoderPort, horizontalEncoderPort, imuPort;
    double horizontalOffset, verticalOffset;
    double horizontalDiam, verticalDiam;

    //control
    double dt; //discretizing interval

    //inverse dynamics
    double kF, kV, kV_r, kA, kA_r, vStatic, vMax;

    //LQR gains
    double qPos, qVel, rTrans;
    double qAng, qAngVel, rRot;

    //LQE gains
    double qPos_f, qVel_f, rTrans_f;
    double qAng_f, qAngVel_f, rRot_f;
};


inline Config toSI(Config config) { //LQR needs SI units
    config.centerToWheel *= INCH;
    config.wheelRadius *= INCH;
    config.horizontalDiam *= INCH;
    config.verticalDiam *= INCH;
    config.horizontalOffset *= INCH;
    config.verticalOffset *= INCH;
    return config;
}