#pragma once
#include "fast_lio/types.hpp"
namespace fastlio {
struct DegeneracyStatus {
  enum Reason { Healthy, Geometry, Insufficient, Residual } reason=Healthy;
  bool degraded=false,severe=false;
  int weak_dimensions=0;
  Eigen::Matrix<double,6,1> eigenvalues=Eigen::Matrix<double,6,1>::Zero();
  Mat6 weak_projector=Mat6::Zero();
  Mat6 information=Mat6::Zero();
  double match_ratio=0,residual=0;
};
class Degeneracy {
 public:
  bool enabled=true;
  double rotation_scale=10., relative_threshold=.01,absolute_threshold=.01;
  double minimum_match_ratio=.05, maximum_residual=.3;
  int enter_frames=2,exit_frames=5,minimum_points=20;
  DegeneracyStatus analyze(const Eigen::MatrixXd& H,double variance,int candidates,double residual,bool commit);
  Eigen::MatrixXd compensate(const Eigen::MatrixXd& H,const DegeneracyStatus& status) const;
  void reset(){bad_=good_=recoverable_good_=0;status_=DegeneracyStatus();}
  const DegeneracyStatus& status() const {return status_;}
 private:
  int bad_=0,good_=0,recoverable_good_=0; DegeneracyStatus status_;
};
}
