#include "simple_tracker.h"

using namespace Eigen;

// 类初始化：定义X和P，以及F，H，
simple_tracker::simple_tracker(Eigen::VectorXd X, Eigen::MatrixXd P){
    this->X_ = X;
    this->P_ = P;

    this->F_  << 1, 0, 0, 0, 0, 0, 0, 1, 0, 0,
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

    this->track_manage.age = 1;
    this->track_manage.unassigned_count = 0;
    this->track_manage.track_status = TRK_Detected;
    this->track_manage.continue_assigned_count = 1;
}

void simple_tracker::trace_predict(double diff_time)
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

void simple_tracker::trace_updateWithDet(Eigen::VectorXd Z)
{
    this->UpdateWithObject(this->X_, this->P_, Z, this->H_, this->R_);
}








