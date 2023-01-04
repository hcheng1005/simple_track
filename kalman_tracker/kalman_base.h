
#pragma once

#ifndef KALMAN_BASE_H
#define KALMAN_BASE_H

#include "eigen3/Eigen/Core"

class kalman_base {
EIGEN_MAKE_ALIGNED_OPERATOR_NEW
public:
    kalman_base(){}
    virtual ~kalman_base() {}
    void Predict(Eigen::VectorXd X, Eigen::MatrixXd P, Eigen::MatrixXd F);
    void UpdateWithObject(Eigen::VectorXd X, Eigen::MatrixXd P, Eigen::MatrixXd H_, Eigen::VectorXd Z);

private:
//    Eigen::MatrixXd F_;
    Eigen::MatrixXd H_;
};





#endif
