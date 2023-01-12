#include "simple_tracker.h"

using namespace Eigen;

// 类初始化：定义X和P，以及F，H，
simple_tracker::simple_tracker(Eigen::VectorXd X, Eigen::MatrixXd P){
    this->X_ = X;
    this->P_ = P;

//    uint dim_states = (uint)this->P_.rows();
//    this->F_ = MatrixXd::Zero(dim_states, dim_states);

    this->F_<<  1, 0, 0, 0, 0, 0, 0, 1, 0, 0,
                0, 1, 0, 0, 0, 0, 0, 0, 1, 0,
                0, 0, 1, 0, 0, 0, 0, 0, 0, 1,
                0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 1, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
                0, 0, 0, 0, 0, 0, 0, 1, 0, 0,
                0, 0, 0, 0, 0, 0, 0, 0, 1, 0,
                0, 0, 0, 0, 0, 0, 0, 0, 0, 1;

   this->H_ <<  1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                0, 1, 0, 0, 0, 0, 0, 0, 0, 0,
                0, 0, 1, 0, 0, 0, 0, 0, 0, 0,
                0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 1, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0, 1, 0, 0, 0;
}

void simple_tracker::trace_update(double diff_time)
{

    this->F_ << 1, 0, 0, 0, 0, 0, 0, diff_time, 0, 0,
                0, 1, 0, 0, 0, 0, 0, 0, diff_time, 0,
                0, 0, 1, 0, 0, 0, 0, 0, 0, diff_time,
                0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 1, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
                0, 0, 0, 0, 0, 0, 0, 1, 0, 0,
                0, 0, 0, 0, 0, 0, 0, 0, 1, 0,
                0, 0, 0, 0, 0, 0, 0, 0, 0, 1;

    this->Predict(this->X_, this->P_, this->F_, this->Q_);

}






