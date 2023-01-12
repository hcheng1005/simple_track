
#ifndef KALMAN_BASE_H
#define KALMAN_BASE_H

#include "eigen3/Eigen/Core"

class kalman_base {
public:
    kalman_base();
    virtual ~kalman_base() = default;

    void Predict(Eigen::VectorXd &X_, Eigen::MatrixXd &P_, Eigen::MatrixXd F_, Eigen::MatrixXd Q_);
    void UpdateWithObject(Eigen::VectorXd &X_, Eigen::MatrixXd &P_, Eigen::VectorXd new_Z, Eigen::MatrixXd H_, Eigen::MatrixXd R_);

private:

};



#endif
