#include "kalman_base.h"


void kalman_base::Predict(Eigen::VectorXd X, Eigen::MatrixXd P, Eigen::MatrixXd F)
{
    X = F * X;
    P = P * F *P.transpose() + Q;
}


void kalman_base::UpdateWithObject(Eigen::VectorXd X, Eigen::MatrixXd P, Eigen::MatrixXd H, Eigen::VectorXd Z)
{
    Eigen::MatrixXd K, S;

    Eigen::
}
