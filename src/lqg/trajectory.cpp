#include "trajectory.hpp"

Trajectory::Trajectory(const trajectory::trajectoryPoint* points, std::size_t count)
        : points(points), count(count) {}


Reference Trajectory::sample(double t) {
    Reference output;
    if(t <= points[0].time) {
        output.state = Eigen::Map<const Vector6d>(points[0].state);
        output.accel = Eigen::Map<const Eigen::Vector3d>(points[0].accel);
        return output;
    }

    if(t >= duration()) {
        Vector6d endState = Eigen::Map<const Vector6d>(points[count - 1].state);
        endState.tail(3).setZero(); //set the terminal vel to 0
        output.state = endState;
        output.accel = Eigen::Vector3d::Zero(); //set the terminal u = 0
        return output;
    }

    if(t < points[carriedIndex].time) carriedIndex = 0; //when the input t is smaller than carry, search from the start
    while(carriedIndex < count - 1 && points[carriedIndex + 1].time < t) carriedIndex++;

    //linear interpolation between the two trajectory points
    const trajectory::trajectoryPoint& pointBehind = points[carriedIndex];
    const trajectory::trajectoryPoint& pointAhead = points[carriedIndex + 1];

    double localT = (t - pointBehind.time) / (pointAhead.time - pointBehind.time);

    Vector6d stateBehind = Eigen::Map<const Vector6d>(pointBehind.state);
    Vector6d stateAhead = Eigen::Map<const Vector6d>(pointAhead.state);

    Eigen::Vector3d accelBehind = Eigen::Map<const Eigen::Vector3d>(pointBehind.accel);
    Eigen::Vector3d accelAhead = Eigen::Map<const Eigen::Vector3d>(pointAhead.accel);

    //lerp
    output.state = stateBehind + localT * (stateAhead - stateBehind);
    output.accel = accelBehind + localT * (accelAhead - accelBehind); 

    output.state(2) = wrapAngle(stateBehind(2) + localT * wrapAngle(stateAhead(2) - stateBehind(2)));

    return output;
}

