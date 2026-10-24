#include "controller.hpp"
#include "trajectory.hpp"

Controller::Controller(Chassis& chassis, Odometry& odom, Config& config)
    : chassis(chassis), odom(odom), config(config) {

    SubsystemModel subModel = doubleIntegrator(config.dt);

    model = fullModel(config.dt);

    Eigen::Matrix2d Qt = Eigen::Vector2d{config.qPos, config.qVel}.asDiagonal();
    Eigen::Matrix2d Qr = Eigen::Vector2d{config.qAng, config.qAngVel}.asDiagonal();

    //LQR gain matrices 
    auto Kt = dareSolver(subModel.A, subModel.B, Qt, config.rTrans);
    auto Kr = dareSolver(subModel.A, subModel.B, Qr, config.rRot);

    Eigen::Matrix2d Qt_f = Eigen::Vector2d{config.qPos_f, config.qVel_f}.asDiagonal();
    Eigen::Matrix2d Qr_f = Eigen::Vector2d{config.qAng_f, config.qAngVel_f}.asDiagonal();

    //LQE gain matrices
    auto Kt_f = dareSolver(subModel.A.transpose(), subModel.C.transpose(), Qt_f, config.rTrans_f);
    auto Kr_f = dareSolver(subModel.A.transpose(), subModel.C.transpose(), Qr_f, config.rRot_f);

    //if any of the DARE solutions failed, rumble the controller and return
    //if this happens, the gains are likely too aggressive that DARE solver failed to converge
    if(!Kt.first || !Kr.first || !Kt_f.first || !Kr_f.first) {
        pros::c::controller_rumble(pros::E_CONTROLLER_MASTER, "---");
        return;
    }

    K = Eigen::Matrix<double, 3, 6>::Zero();
    K(0, 0) = Kt.second(0); //x
    K(1, 1) = Kt.second(0); //y
    K(2, 2) = Kr.second(0); //theta
    K(0, 3) = Kt.second(1); //xdot
    K(1, 4) = Kt.second(1); //ydot
    K(2, 5) = Kr.second(1); //thetadot

    Kf = Eigen::Matrix<double, 6, 3>::Zero();
    Kf(0, 0) = Kt_f.second(0); //x
    Kf(1, 1) = Kt_f.second(0); //y
    Kf(2, 2) = Kr_f.second(0); //theta
    Kf(3, 0) = Kt_f.second(1); //xdot
    Kf(4, 1) = Kt_f.second(1); //ydot
    Kf(5, 2) = Kr_f.second(1); //thetadot
}




void Controller::followTrajectory(const trajectory::PathData& trajectory, bool async) {
    cancelRequested = false;
    motionRunning = true;

    //if async = true, run the trajectory in a separate task and return immediately
    if(async) {
        pros::Task task([this, trajectory] () {followTrajectory(trajectory, false);});
        pros::delay(10);
        return;
    }

    Trajectory currentTraj(trajectory.points, trajectory.count);

    std::uint32_t startTime = pros::millis();
    std::uint32_t prevTime = startTime;

    double elapsed = 0.0;
    while(elapsed < currentTraj.duration() && !cancelRequested) {
        elapsed = (pros::millis() - startTime) / 1000.0;

        Reference reference = currentTraj.sample(elapsed);

        //get the current filtered state estimate
        motionMutex.take();
        Vector6d xHatCurrent = xHat;
        motionMutex.give();

        Vector6d xError = xHatCurrent - reference.state;
        xError(2) = wrapAngle(xError(2)); //wrap the angle error to [-pi, pi]

        Eigen::Vector3d u = reference.accel - K * xError; //control law
        Eigen::Vector3d desiredVel {reference.state(3), reference.state(4), reference.state(5)};

        Eigen::Matrix3d R; //rotation matrix to convert from global to local frame
        double theta = xHatCurrent(2);
        R << 
        std::cos(theta), std::sin(theta), 0,
        -std::sin(theta), std::cos(theta), 0,
        0, 0, 1;

        //apply control input in local frame
        Eigen::Vector3d localApplied = R.transpose() * chassis.applyLocalWrench(R * u, R * desiredVel);

        //store the applied control input for the kalman filter update
        motionMutex.take();
        uApplied = localApplied;
        motionMutex.give();

        pros::Task::delay_until(&prevTime, config.dt * 1000);
    }

    chassis.stop();

    motionMutex.take();
    //reset the applied control input when the motion is complete 
    uApplied = Eigen::Vector3d::Zero();
    motionMutex.give();

    //reset the motion state
    motionRunning = false;
    cancelRequested = false;
}


void Controller::cancelMotion() {
    if(!isInMotion()) {
        return;
    }
    cancelRequested = true;
}

void Controller::runMotion() {
    Eigen::Vector3d initState = odom.getState();
    xHat <<
    initState(0),
    initState(1),
    initState(2),
    0.0,
    0.0,
    0.0;

    motionTask.emplace([this]() { this->updateMotion(); });
}

void Controller::waitUntilDone() {
    while (isInMotion()) {
        pros::delay(10);
    }
}

void Controller::updateMotion() {
    std::uint32_t prev = pros::millis();
    while(true) {

        //get current sensor readings
        Eigen::Vector3d y = odom.getState();

        motionMutex.take();

        //compute the difference between the measured state and the predicted state
        Eigen::Vector3d residual = y - model.C * xHat;
        residual(2) = wrapAngle(residual(2));

        Eigen::Vector3d u = uApplied;

        //update the filtered state
        xHat = model.A * xHat + model.B * uApplied + Kf * residual;

        motionMutex.give();

        pros::Task::delay_until(&prev, config.dt * 1000);
    }
}