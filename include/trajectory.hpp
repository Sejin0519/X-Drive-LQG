#pragma once
#include "Eigen/Dense"
#include "paths.hpp"
#include "types.hpp"
#include "util.hpp"

struct Reference {
    Vector6d state;
    Eigen::Vector3d accel;
};

class Trajectory {
public:
    Trajectory(const trajectory::trajectoryPoint* pts, std::size_t count);
    Reference sample(double t);
    double duration() const {return points[count - 1].time;}

private:
    const trajectory::trajectoryPoint* points;
    std::size_t count;
    std::size_t carriedIndex = 0;
};