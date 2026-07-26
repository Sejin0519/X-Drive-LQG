#include "controller.hpp"
#include "trajectory.hpp"

Controller::Controller(Chassis& chassis, Odometry& odom, Config& config) 
    : chassis(chassis), odom(odom), config(config) {
    SubsystemModel transModel = translationalModel(config.mass, config.dt);
    SubsystemModel rotModel = rotationalModel(config.inertia, config.dt);
    Eigen::Matrix2d Qt = Eigen::Vector2d{config.qPos, config.qVel}.asDiagonal();
    Eigen::Matrix2d Qr = Eigen::Vector2d{config.qAng, config.qAngVel}.asDiagonal();
    auto Kt = dareSolver(transModel.A, transModel.B, Qt, config.rTrans);
    auto Kr = dareSolver(rotModel.A, rotModel.B, Qr, config.rRot);

    if(!Kt.first || !Kr.first) {
        return; //TODO: make controller rumble
    }

    K = Eigen::Matrix<double, 3, 6>::Zero();
    K.block<1, 2>(0, 0) = Kt.second;
    K.block<1, 2>(1, 2) = Kt.second;
    K.block<1, 2>(2, 4) = Kr.second;
}

// void Controller::holdPose(const Pose& target) {
//     while(true) {
//         Eigen::Matrix<double, 6, 1> state = odom.getState();
//         Eigen::Matrix<double, 6, 1> state = 
        
//     }
// }



void Controller::followTrajectory(Trajectory& traj, bool async) {
    if(async) {
        pros::Task task([&] () {followTrajectory(traj, false);});
        pros::delay(10);
        return;
    }
    cancelRequested = false;
    double duration = traj.duration() * 1000; //sec to millis
    double startTime = pros::millis();
    while((pros::millis() - startTime < duration) && !cancelRequested) {
        Reference reference = traj.sample(pros::millis() - startTime);
        Vector6d x = odom.getState();
        Vector6d xError = reference.state - x;
        
        Eigen::Vector3d u = -K * xError;
        
        Eigen::Matrix3d rotationMatrix;
        double theta = x(2);
        rotationMatrix << std::cos(theta), std::sin(theta), 0,
        -std::sin(theta), std::cos(theta), 0,
        0, 0, 1;

        Eigen::Vector3d localForce = rotationMatrix * u;

        chassis.applyLocalForce(localForce);
        pros::delay(10);
    }
    chassis.stop();

}

void Controller::cancelMotion() {
    cancelRequested = true;
}