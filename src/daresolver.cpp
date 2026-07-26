#include "dareSolver.hpp"


std::pair<bool, Eigen::Matrix<double, 1, 2>> dareSolver(const Eigen::Matrix2d& A, const Eigen::Vector2d& B, 
    const Eigen::Matrix2d& Q, double R, double tolerance, int maxIteration) {

    Eigen::Matrix2d P = Eigen::Matrix2d::Zero();
    Eigen::Matrix2d P_next = Q; //initialize the terminal P_N to Q
    Eigen::Matrix<double, 1, 2> K = Eigen::Matrix<double, 1, 2>::Zero();

    for(int i = 0; i < maxIteration; i++) { //backward iteration
        double S = (B.transpose() * P_next * B).value() + R; //(Vector)^T * (Vector) is a scalar
        K = (1.0 / S) * B.transpose() * P_next * A;
        P = Q + K.transpose() * R * K + (A - B * K).transpose() * P_next * (A - B * K);

        if((P - P_next).norm() < tolerance) {
            return {true, K};
        }
        P_next = P;
    }
    return {false, K}; //convergence failed
}