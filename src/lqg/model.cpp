#include "model.hpp"

SubsystemModel doubleIntegrator(double dt) {
    //ZOH discretization of a double integrator system
    Eigen::Matrix2d A;
    A << 1.0, dt,
    0.0, 1.0;
    Eigen::Vector2d B;
    B << dt * dt / 2.0, dt;
    return {A, B};
}

FullModel fullModel(double dt) {
    //ZOH discretization of a double integrator system in 2D with rotation
    FullModel model;
    model.A.setZero();
    model.B.setZero();
    model.C.setZero();

    model.A.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
    model.A.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity() * dt;
    model.A.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity();

    model.B.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * dt * dt / 2.0;
    model.B.block<3, 3>(3, 0) = Eigen::Matrix3d::Identity() * dt;

    model.C.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();


    return {model.A, model.B, model.C};
}