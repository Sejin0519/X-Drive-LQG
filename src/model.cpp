#include "model.hpp"

//ZOH discretizzation
SubsystemModel translationalModel(double mass, double dt) {
    Eigen::Matrix2d A;
    Eigen::Vector2d B;
    A << 1.0, dt,
    0.0, 1.0;
    B << (dt * dt) / (2.0 * mass), 
    dt / mass;
    return {A, B};
}

//same thing with different names
SubsystemModel rotationalModel(double inertia, double dt) {
    Eigen::Matrix2d A;
    Eigen::Vector2d B;
    A << 1.0, dt,
    0.0, 1.0;
    B << (dt * dt) / (2.0 * inertia), 
    dt / inertia;

    return {A, B};
}