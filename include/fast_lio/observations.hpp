#pragma once
#include "fast_lio/types.hpp"
#include <optional>
namespace fastlio {
struct LinearObservation {
  Eigen::MatrixXd H,R;
  Eigen::VectorXd residual;
  // Optional diagonal noise avoids allocating an image-sized dense matrix.
  Eigen::VectorXd variances;
};
struct WheelModel {
  Pose imu_T_base=Pose::Identity(); bool nonholonomic=true;
  double lateral_variance=.25,vertical_variance=.25;
  LinearObservation linearize(const State& state,const Wheel& wheel,const Vec3& angular_rate) const;
};
LinearObservation positionObservation(const State& state,const Fix& fix,const Vec3& antenna);
class Geodesy {
 public:
  Vec3 convert(double lat,double lon,double altitude);
  void reset(){initialized_=false;}
 private:
  bool initialized_=false;Vec3 origin_;Mat3 ecef_to_enu_;
};
class GnssAlignment {
 public:
  double yaw=0,minimum_baseline=5.;bool yaw_known=false;
  std::optional<Fix> align(const Fix& enu,const State& s,const Vec3& lever);
  void reset(){initialized_=false;points_.clear();}
 private:
  bool initialized_=false;Mat3 rotation_;Vec3 offset_;
  std::vector<std::pair<Vec3,Vec3>> points_;
};
}
