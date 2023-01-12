
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




class simple_tracker : public kalman_base {

public:
    simple_tracker(Eigen::VectorXd X, Eigen::MatrixXd P);

    virtual ~simple_tracker(){}

    void trace_update(double diff_time);

    Eigen::VectorXd X_;
    Eigen::MatrixXd P_;
    Eigen::MatrixXd F_;
    Eigen::MatrixXd H_;

    uint age;


private:
    Eigen::MatrixXd Q_;
    Eigen::MatrixXd R_;

};



#endif
