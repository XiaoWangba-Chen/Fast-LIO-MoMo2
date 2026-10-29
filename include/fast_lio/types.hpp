#pragma once
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <deque>
#include <vector>
#include <string>
#include <cstdint>
#include <stdexcept>
#include <cmath>
namespace fastlio {
using Vec3 = Eigen::Vector3d;
using Mat3 = Eigen::Matrix3d;
using Mat6 = Eigen::Matrix<double,6,6>;
using Cov = Eigen::Matrix<double,23,23>;
using Pose = Eigen::Isometry3d;
inline Mat3 skew(const Vec3& v) { Mat3 a; a << 0,-v.z(),v.y(),v.z(),0,-v.x(),-v.y(),v.x(),0; return a; }
inline Mat3 expSO3(const Vec3& v) {
  const double n=v.norm();
  if(n<1e-10) return Mat3::Identity()+skew(v)+0.5*skew(v)*skew(v);
  return Eigen::AngleAxisd(n,v/n).toRotationMatrix();
}
inline Mat3 rightJacobian(const Vec3& v) {
  const double n=v.norm(); const Mat3 a=skew(v);
  if(n<1e-6) return Mat3::Identity()-0.5*a+a*a/6.;
  return Mat3::Identity()-(1-std::cos(n))/(n*n)*a+(n-std::sin(n))/(n*n*n)*a*a;
}
struct State {
  double time=-1; uint64_t version=0;
  Pose pose=Pose::Identity();
  Vec3 velocity=Vec3::Zero(), bg=Vec3::Zero(), ba=Vec3::Zero(), gravity{0,0,-9.81};
  Vec3 angular_rate=Vec3::Zero();
  Eigen::Matrix<double,3,2> gravity_basis=Eigen::Matrix<double,3,2>::Zero();
  Cov covariance=Cov::Identity();
};
struct Imu { double time; Vec3 acceleration, angular_velocity; };
struct Noise { double gyro=.1, accel=.1, gyro_bias=.0001, accel_bias=.0001; };
enum class Side { Frontend, Backend };
inline Side parseSide(const std::string& s) {
  if(s=="frontend") return Side::Frontend;
  if(s=="backend") return Side::Backend;
  throw std::invalid_argument("fusion.mode must be frontend or backend");
}
struct Audit {
  uint64_t accepted=0,rejected=0,late=0,overflow=0;
};
struct Wheel { double time=0,speed=0,variance=.04; };
struct Fix { double time=0; Vec3 position=Vec3::Zero(); Mat3 covariance=Mat3::Identity(); };
struct Pixel { uint64_t feature=0; Eigen::Vector3d uv; }; // u_left, v_left, u_right (rectified)
struct StereoFrame { double time=0; uint64_t id=0; std::vector<Pixel> pixels; };
struct Camera {
  double fx=0,fy=0,cx=0,cy=0,baseline=0,pixel_sigma=1.;
  Pose body_T_camera=Pose::Identity();
  bool valid() const {return fx>0&&fy>0&&baseline>0&&std::isfinite(fx)&&std::isfinite(fy)&&std::isfinite(cx)&&std::isfinite(cy)&&std::isfinite(baseline);}
  Vec3 triangulate(const Vec3& uv) const {
    double z=fx*baseline/(uv.x()-uv.z());
    return Vec3((uv.x()-cx)*z/fx,(uv.y()-cy)*z/fy,z);
  }
  Vec3 project(const Vec3& p) const {
    return Vec3(fx*p.x()/p.z()+cx,fy*p.y()/p.z()+cy,fx*(p.x()-baseline)/p.z()+cx);
  }
};
struct ObservationId {
  uint64_t session=0,sequence=0; uint8_t sensor=0;
};
} // namespace fastlio
