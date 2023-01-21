
#ifndef SIMPLE_TRACKER_H
#define SIMPLE_TRACKER_H

#include "./kalman_tracker/kalman_base.h"
/*
running:
  covariance: default
  score_threshold: 0.5
  max_age_since_update: 2
  min_hits_to_birth: 3
  match_type: bipartite
  asso: giou
  has_velo: false
  motion_model: kf
  asso_thres:
    iou: 0.9
    giou: 1.5

redundancy:
  mode: mm
  det_score_threshold:
    iou: 0.1
    giou: 0.1
  det_dist_threshold:
    iou: 0.1
    giou: -0.5

data_loader:
  pc: true
  nms: true
  nms_thres: 0.25
*/

/*
  X = [bbox.x, bbox.y, bbox.z, bbox.o, bbox.l, bbox.w, bbox.h, vx, vy, vz]
  Z = [bbox.x, bbox.y, bbox.z, bbox.o, bbox.l, bbox.w, bbox.h]
*/


#define X_DIM   (10)
#define Z_DIM   (7)

typedef struct dets_t
{
    double x;
    double y;
    double z;
    double heading;
    double length;
    double width;
    double height;
    double score;

    bool valid = true;
}dets_t;


typedef enum
{
    TRK_Invalid = 0,
    TRK_Detected,
    TRK_Confirmed,
    TRK_Delete
}trace_status_enum;

typedef struct
{
    uint32_t age = 0;
    uint8_t continue_assigned_count = 0;
    uint8_t unassigned_count = 0;
    trace_status_enum track_status = TRK_Invalid;
}trace_manage_struct;


class simple_tracker : public kalman_base {

public:
    simple_tracker(Eigen::VectorXd X, Eigen::MatrixXd P);

    virtual ~simple_tracker(){}

    void trace_predict(double diff_time);
    void trace_updateWithDet(Eigen::VectorXd Z);

    Eigen::VectorXd X_ = Eigen::VectorXd(X_DIM);
    Eigen::MatrixXd P_ = Eigen::MatrixXd(X_DIM, X_DIM);
    Eigen::MatrixXd F_ = Eigen::MatrixXd(X_DIM, X_DIM);
    Eigen::MatrixXd H_ = Eigen::MatrixXd(Z_DIM, X_DIM);

    trace_manage_struct track_manage;

private:
    Eigen::MatrixXd Q_ = Eigen::MatrixXd::Identity(X_DIM, X_DIM) * 1.0;
    Eigen::MatrixXd R_ = Eigen::MatrixXd::Identity(Z_DIM, Z_DIM) * 2.0;


};



#endif
