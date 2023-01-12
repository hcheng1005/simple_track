#include "kalman_base.h"
#include "eigen3/Eigen/Dense"

using namespace Eigen;

kalman_base::kalman_base()
{

}

void kalman_base::Predict(Eigen::VectorXd &X_, Eigen::MatrixXd &P_, Eigen::MatrixXd F_, Eigen::MatrixXd Q_)
{
    X_ = F_ * X_;
    P_ = F_ * P_ *F_.transpose() + Q_;
}


void kalman_base::UpdateWithObject(Eigen::VectorXd &X_, Eigen::MatrixXd &P_, Eigen::VectorXd new_Z, Eigen::MatrixXd H_, Eigen::MatrixXd R_)
{
    MatrixXd K, S;

    MatrixXd z_pre = H_ * X_;
    S = H_ * P_ * H_.transpose() + R_;
    K = P_ * H_ * S.inverse();

    X_ = X_ + K * (new_Z - z_pre);
    P_ = (Eigen::MatrixXd::Identity(X_.rows(), X_.rows()) - K * H_) * P_;

}
