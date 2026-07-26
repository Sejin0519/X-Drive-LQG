#pragma once
#include <cmath>

struct Pose {
    double x = 0.0;
    double y = 0.0;
    double theta = 0.0;
};

inline double degToRad(double degrees) {
    return degrees * (M_PI / 180);
}

inline double radToDeg(double radians) {
    return radians * (180.0 / M_PI);
}

inline double wrapAngle(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle <= -M_PI) angle += 2.0 * M_PI;
    return angle;
} //-pi to pi

