#include "odom.hpp"

Odometry::Odometry(const Config& config)
    : config(config),
      verticalEncoder(pros::Rotation(config.verticalEncoderPort)),
      horizontalEncoder(pros::Rotation(config.horizontalEncoderPort)),
      imu(pros::Imu(config.imuPort))
{}

void Odometry::initialize() {
    imu.reset(true);

    horizontalEncoder.reset_position();
    verticalEncoder.reset_position();

    pros::c::controller_rumble(pros::E_CONTROLLER_MASTER, ".");
    
    setPose(0, 0, 0);
    saveTime = pros::millis() / 1000.0;

    odomTask.emplace([this]() { this->update(); });
}

Pose Odometry::getPose(bool radians){
    mutex.take();
    Pose currentPose = pose;
    mutex.give();
    if(!radians) currentPose.theta = -radToDeg(currentPose.theta);
    return currentPose;
}

Vector6d Odometry::getState() {
    mutex.take();
    Pose currentPose = pose;
    Pose currentVel = velocity;
    mutex.give();
    Vector6d currentState = {currentPose.x, currentPose.y, currentPose.theta, currentVel.x, currentVel.y, currentVel.theta};
    return currentState;
}

void Odometry::setPose(Pose newPose) {
    mutex.take();
    pose = newPose;
    prevPose = newPose;
    imu.set_rotation(-radToDeg(newPose.theta));
    const Eigen::Vector3d& readings = getSensorReadings();
    prevHorizontalDist = readings(0);
    prevVerticalDist = readings(1);
    prevTheta = newPose.theta;
    mutex.give();
}


void Odometry::setPose(double x, double y, double theta) {
    Pose newPose = {x, y, theta};
    setPose(newPose);
}

Eigen::Vector3d Odometry::getSensorReadings() {
    Eigen::Vector3d readings;
    double horizontalPosition = horizontalEncoder.get_position();
    double verticalPosition = verticalEncoder.get_position();

    double horizontalDist = horizontalPosition / 36000.0 * M_PI * config.horizontalDiam;
    double verticalDist = verticalPosition / 36000.0 * M_PI * config.verticalDiam;
    double theta = wrapAngle(-degToRad(imu.get_rotation()));

    readings << horizontalDist, verticalDist, theta;
    return readings;
}


void Odometry::update() {

    while(true) {
        mutex.take();
        Eigen::Vector3d readings = getSensorReadings();

        double deltaHorizontal = readings(0) - prevHorizontalDist;
        double deltaVertical = readings(1) - prevVerticalDist;
        double deltaTheta = wrapAngle(readings(2) - prevTheta);

        prevHorizontalDist = readings(0);
        prevVerticalDist = readings(1);
        prevTheta = readings(2);

        double midTheta = pose.theta + deltaTheta / 2.0;
        double localDeltaX, localDeltaY;


        if(std::fabs(deltaTheta) < 1e-9) {
            localDeltaX = deltaHorizontal;
            localDeltaY = deltaVertical;
        } else {
            localDeltaX = 2.0 * std::sin(deltaTheta / 2.0) * (deltaHorizontal / deltaTheta + config.horizontalOffset);
            localDeltaY = 2.0 * std::sin(deltaTheta / 2.0) * (deltaVertical / deltaTheta + config.verticalOffset);
        }



        pose.x += std::cos(midTheta) * localDeltaX - std::sin(midTheta) * localDeltaY;
        pose.y += std::sin(midTheta) * localDeltaX + std::cos(midTheta) * localDeltaY;
        pose.theta = readings(2);

        double deltaT = pros::millis() / 1000.0 - saveTime;

        if(deltaT > 0.0) {
            velocity.x = (pose.x - prevPose.x) / deltaT;
            velocity.y = (pose.y - prevPose.y) / deltaT;
            velocity.theta = wrapAngle(pose.theta - prevPose.theta) / deltaT;

            prevPose = pose;
            saveTime += deltaT;
        }

        mutex.give();
        pros::delay(10);
    }
}