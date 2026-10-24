#pragma once
#include "Eigen/Dense"
#include "paths.hpp"
#include "util.hpp"

struct Reference {
    Vector6d state;
    Eigen::Vector3d accel;
};

class Trajectory {
public:
    Trajectory(const trajectory::trajectoryPoint* points, std::size_t count);

    /**
     * @brief Get the reference state and acceleration at a given time.
     * Uses linear interpolation between trajectory points to compute the reference.
     *
     * @param t The time at which to sample the trajectory.
     * @return Reference containing the state and acceleration at time t.
     */
    Reference sample(double t);

    /**
     * @brief Get the total duration of the trajectory.
     *
     * @return The time of the last trajectory point.
     */
    double duration() const {return points[count - 1].time;}

private:
    const trajectory::trajectoryPoint* points;
    std::size_t count;
    std::size_t carriedIndex = 0;
};