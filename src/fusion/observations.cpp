#include "fast_lio/observations.hpp"
namespace fastlio {
LinearObservation WheelModel::linearize(const State& s,const Wheel& w,const Vec3& angular_rate) const {
  const Mat3 R=s.pose.linear(),B=imu_T_base.linear().transpose();
  Vec3 vb=R.transpose()*s.velocity;
  Vec3 prediction=B*(vb+(angular_rate-s.bg).cross(imu_T_base.translation()));
  const int rows=nonholonomic?3:1;
  LinearObservation o; o.H=Eigen::MatrixXd::Zero(rows,23); o.R=Eigen::MatrixXd::Zero(rows,rows);
  Eigen::Matrix<double,3,23> H=Eigen::Matrix<double,3,23>::Zero();
  H.block<3,3>(0,3)=B*skew(vb);H.block<3,3>(0,12)=B*R.transpose();
  H.block<3,3>(0,15)=B*skew(imu_T_base.translation());
  o.H=H.topRows(rows);o.residual=(Vec3(w.speed,0,0)-prediction).head(rows);
  o.R(0,0)=w.variance;
  if(nonholonomic){o.R(1,1)=lateral_variance;o.R(2,2)=vertical_variance;}
  return o;
}
LinearObservation positionObservation(const State& s,const Fix& f,const Vec3& antenna) {
  LinearObservation o;o.H=Eigen::MatrixXd::Zero(3,23);
  o.H.leftCols<3>()=Mat3::Identity();o.H.block<3,3>(0,3)=-s.pose.linear()*skew(antenna);
  o.residual=f.position-s.pose*antenna;o.R=f.covariance;return o;
}
Vec3 Geodesy::convert(double lat,double lon,double altitude) {
  if(!std::isfinite(lat)||!std::isfinite(lon)||!std::isfinite(altitude)||std::abs(lat)>90||std::abs(lon)>180)throw std::invalid_argument("Invalid geodetic coordinate");
  const double phi=lat*M_PI/180,lambda=lon*M_PI/180,a=6378137.,e2=6.6943799901413165e-3;
  const double N=a/std::sqrt(1-e2*std::sin(phi)*std::sin(phi));
  Vec3 ecef((N+altitude)*std::cos(phi)*std::cos(lambda),(N+altitude)*std::cos(phi)*std::sin(lambda),(N*(1-e2)+altitude)*std::sin(phi));
  if(!initialized_){origin_=ecef;ecef_to_enu_<<-std::sin(lambda),std::cos(lambda),0,-std::sin(phi)*std::cos(lambda),-std::sin(phi)*std::sin(lambda),std::cos(phi),std::cos(phi)*std::cos(lambda),std::cos(phi)*std::sin(lambda),std::sin(phi);initialized_=true;}
  return ecef_to_enu_*(ecef-origin_);
}
std::optional<Fix> GnssAlignment::align(const Fix& enu,const State& s,const Vec3& lever) {
  if(!initialized_) {
    if(!s.gravity.allFinite()||s.gravity.norm()<1e-6)return {};
    const Mat3 level=Eigen::Quaterniond::FromTwoVectors(Vec3::UnitZ(),-s.gravity.normalized()).toRotationMatrix();
    Vec3 local=s.pose*lever;
    points_.emplace_back(enu.position,local);
    if(!yaw_known) {
      if(points_.size()<2)return {};
      if((points_.back().first-points_.front().first).head<2>().norm()<minimum_baseline||
         (level.transpose()*(local-points_.front().second)).head<2>().norm()<minimum_baseline) {if(points_.size()>500)points_.erase(points_.begin()+1);return {};}
      Vec3 a=enu.position-points_.front().first,b=level.transpose()*(local-points_.front().second);
      if(std::abs(a.head<2>().norm()-b.head<2>().norm())>std::max(2.,.3*a.head<2>().norm())) {points_.clear();return {};}
      yaw=std::atan2(b.y(),b.x())-std::atan2(a.y(),a.x());
    }
    rotation_=level*Eigen::AngleAxisd(yaw,Vec3::UnitZ()).toRotationMatrix();
    offset_=points_.front().second-rotation_*points_.front().first;initialized_=true;points_.clear();
  }
  Fix result=enu;result.position=rotation_*enu.position+offset_;result.covariance=rotation_*enu.covariance*rotation_.transpose();return result;
}
}
