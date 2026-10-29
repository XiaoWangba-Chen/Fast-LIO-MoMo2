#include "fast_lio/propagator.hpp"
namespace fastlio {
bool Propagator::integrate(State& s,const Imu& a,const Imu& b,const Noise& n,double scale,double gap) {
  double dt=b.time-s.time;
  if(dt<=0||dt>gap||b.time-a.time>gap||a.time>s.time+1e-8) return false;
  double alpha=(s.time-a.time)/std::max(1e-12,b.time-a.time);
  Vec3 omega=(.5*(1-alpha)*a.angular_velocity+.5*(1+alpha)*b.angular_velocity)-s.bg;
  Vec3 acc=scale*(.5*(1-alpha)*a.acceleration+.5*(1+alpha)*b.acceleration)-s.ba;
  const Mat3 R=s.pose.linear(),mid=R*expSO3(omega*dt*.5);
  Vec3 world_acc=mid*acc+s.gravity;
  s.pose.translation()+=s.velocity*dt+.5*world_acc*dt*dt;
  s.velocity+=world_acc*dt; s.pose.linear()=R*expSO3(omega*dt);
  Cov F=Cov::Identity();
  F.block<3,3>(0,12)=Mat3::Identity()*dt;
  F.block<3,3>(0,3)=-.5*mid*skew(acc)*dt*dt;
  F.block<3,3>(0,18)=-.5*mid*dt*dt;
  F.block<3,3>(3,3)=expSO3(-omega*dt);
  F.block<3,3>(3,15)=-Mat3::Identity()*dt;
  F.block<3,3>(12,3)=-mid*skew(acc)*dt;
  F.block<3,3>(12,18)=-mid*dt;
  // Gravity occupies a two-dimensional tangent plane in the FAST-LIO state.
  Vec3 axis=s.gravity.normalized(); Vec3 e1=axis.unitOrthogonal(),e2=axis.cross(e1);
  Eigen::Matrix<double,3,2> G; G<<e1,e2; G*=s.gravity.norm();
  if(s.gravity_basis.norm()>0)G=s.gravity_basis;
  F.block<3,2>(12,21)=G*dt; F.block<3,2>(0,21)=.5*G*dt*dt;
  Eigen::Matrix<double,23,12> B=Eigen::Matrix<double,23,12>::Zero();
  B.block<3,3>(3,0)=-Mat3::Identity()*dt;
  B.block<3,3>(12,3)=-mid*dt; B.block<3,3>(0,3)=-.5*mid*dt*dt;
  B.block<3,3>(15,6)=Mat3::Identity()*dt; B.block<3,3>(18,9)=Mat3::Identity()*dt;
  Eigen::Matrix<double,12,1> q; q<<Vec3::Constant(n.gyro),Vec3::Constant(n.accel),Vec3::Constant(n.gyro_bias),Vec3::Constant(n.accel_bias);
  s.covariance=F*s.covariance*F.transpose()+B*q.asDiagonal()*B.transpose();
  s.covariance=(s.covariance+s.covariance.transpose()).eval()*.5;
  s.time=b.time;s.angular_rate=b.angular_velocity; return s.pose.matrix().allFinite()&&s.covariance.allFinite();
}
std::optional<State> Propagator::push(const Imu& imu) {
  std::lock_guard<std::mutex> lock(mutex_);
  if(!imu.acceleration.allFinite()||!imu.angular_velocity.allFinite()||!std::isfinite(imu.time))return {};
  if(!history_.empty()&&imu.time<=history_.back().time) return {};
  history_.push_back(imu);
  while(history_.size()>max_samples)history_.pop_front();
  if(!ready_||history_.size()<2||imu.time<=state_.time)return {};
  if(!integrate(state_,history_[history_.size()-2],imu,noise,acceleration_scale,max_gap)) {ready_=false;++gaps_;return {};}
  if(state_.time<=last_published_)return {};
  last_published_=state_.time;return state_;
}
bool Propagator::correct(const State& anchor) {
  std::lock_guard<std::mutex> lock(mutex_);
  if(anchor.time<anchor_time_||(anchor.time==anchor_time_&&anchor.version<=anchor_version_))return false;
  anchor_time_=anchor.time; anchor_version_=anchor.version; state_=anchor; ready_=true;
  while(history_.size()>2&&history_[1].time<=anchor.time)history_.pop_front();
  if(!history_.empty()&&history_.front().time>anchor.time+1e-8)ready_=false;
  for(size_t i=1;ready_&&i<history_.size();++i) {
    if(history_[i].time<=state_.time)continue;
    if(!integrate(state_,history_[i-1],history_[i],noise,acceleration_scale,max_gap)) {ready_=false;++gaps_;}
  }
  return ready_;
}
void Propagator::reset(){std::lock_guard<std::mutex> lock(mutex_);history_.clear();state_=State();ready_=false;last_published_=anchor_time_=-1;anchor_version_=0;gaps_=0;}
}
