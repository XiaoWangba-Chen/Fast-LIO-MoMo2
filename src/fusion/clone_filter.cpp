#include "fast_lio/clone_filter.hpp"
#include <Eigen/Cholesky>
#include <Eigen/SVD>
#include <Eigen/QR>
#include <set>
namespace fastlio {
void CloneFilter::transportCurrent(const Cov& J){
  Eigen::MatrixXd T=Eigen::MatrixXd::Identity(covariance_.rows(),covariance_.cols());T.topLeftCorner<23,23>()=J;covariance_=T*covariance_*T.transpose();
}
void CloneFilter::reset(const Cov& P){covariance_=P;clones_.clear();tracks_.clear();}
void CloneFilter::predict(const Cov& F,const Cov& P){
  int c=covariance_.rows()-23;
  if(c>0){covariance_.topRightCorner(23,c)=(F*covariance_.topRightCorner(23,c)).eval();covariance_.bottomLeftCorner(c,23)=covariance_.topRightCorner(23,c).transpose();}
  covariance_.topLeftCorner<23,23>()=P;
}
void CloneFilter::condition(const Cov& prior,const Cov& posterior,const Eigen::Matrix<double,23,1>& dx,const Cov& J){
  int n=covariance_.rows()-23;
  if(n>0){
    Eigen::MatrixXd A=prior.ldlt().solve(covariance_.topRightCorner(23,n));
    Eigen::VectorXd dc=A.transpose()*dx;
    covariance_.bottomRightCorner(n,n)+=A.transpose()*(J.inverse()*posterior*J.inverse().transpose()-prior)*A;
    covariance_.topRightCorner(23,n)=posterior*J.inverse().transpose()*A;
    covariance_.bottomLeftCorner(n,23)=covariance_.topRightCorner(23,n).transpose();
    for(size_t i=0;i<clones_.size();++i){clones_[i].pose.translation()+=dc.segment<3>(6*i);clones_[i].pose.linear()*=expSO3(dc.segment<3>(6*i+3));}
    Eigen::MatrixXd reset=Eigen::MatrixXd::Identity(23+n,23+n);
    for(size_t i=0;i<clones_.size();++i)reset.block<3,3>(26+6*i,26+6*i)=rightJacobian(dc.segment<3>(6*i+3));
    covariance_=reset*covariance_*reset.transpose();
  }
  covariance_.topLeftCorner<23,23>()=posterior;
  covariance_=(covariance_+covariance_.transpose()).eval()*.5;
}
void CloneFilter::clone(uint64_t id,const State& s){
  const int n=covariance_.rows();Eigen::MatrixXd J=Eigen::MatrixXd::Zero(6,n);J.leftCols<6>()=Mat6::Identity();
  Eigen::MatrixXd P=Eigen::MatrixXd::Zero(n+6,n+6);P.topLeftCorner(n,n)=covariance_;
  P.bottomLeftCorner(6,n)=J*covariance_;P.topRightCorner(n,6)=P.bottomLeftCorner(6,n).transpose();
  P.bottomRightCorner<6,6>()=J*covariance_*J.transpose();covariance_=P;
  clones_.push_back(Clone{id,s.time,s.pose});
}
std::optional<Correction> CloneFilter::update(const LinearObservation& o,double gate){
  if(o.H.rows()==0)return {};
  const int n=covariance_.rows();int m=o.H.rows();
  Eigen::MatrixXd H=Eigen::MatrixXd::Zero(m,n);H.leftCols(o.H.cols())=o.H;
  Eigen::MatrixXd R=o.R;Eigen::VectorXd residual=o.residual;
  const bool diagonal_noise=o.variances.size()==m;
  if(diagonal_noise&&(o.variances.array()<=0).any())return {};
  if(diagonal_noise&&m<=n)R=o.variances.asDiagonal();
  double discarded=0;const int original_rows=m;
  if(m>n){
    if(diagonal_noise||R.isDiagonal()){for(int i=0;i<m;++i){double sigma=std::sqrt(diagonal_noise?o.variances[i]:R(i,i));if(!(sigma>0))return {};H.row(i)/=sigma;residual[i]/=sigma;}}
    else {Eigen::LLT<Eigen::MatrixXd> noise(R);if(noise.info()!=Eigen::Success)return {};H=noise.matrixL().solve(H).eval();residual=noise.matrixL().solve(residual).eval();}
    Eigen::HouseholderQR<Eigen::MatrixXd> qr(H);
    Eigen::VectorXd projected=qr.householderQ().adjoint()*residual;
    discarded=projected.tail(m-n).squaredNorm();
    H=(qr.householderQ().adjoint()*H).topRows(n).eval();residual=projected.head(n);m=n;R=Eigen::MatrixXd::Identity(n,n);
  }
  Eigen::MatrixXd S=H*covariance_*H.transpose()+R;
  Eigen::LDLT<Eigen::MatrixXd> solve(S);
  if(solve.info()!=Eigen::Success||!solve.isPositive()||!residual.allFinite()||residual.dot(solve.solve(residual))+discarded>gate*original_rows)return {};
  const Eigen::MatrixXd K=solve.solve(H*covariance_).transpose();
  const Eigen::VectorXd dx=K*residual;
  const Eigen::MatrixXd I=Eigen::MatrixXd::Identity(n,n)-K*H;
  covariance_=I*covariance_*I.transpose()+K*R*K.transpose();
  Eigen::MatrixXd J=Eigen::MatrixXd::Identity(n,n);
  for(int index:{3,6})J.block<3,3>(index,index)=rightJacobian(dx.segment<3>(index));
  for(size_t i=0;i<clones_.size();++i){
    clones_[i].pose.translation()+=dx.segment<3>(23+6*i);
    clones_[i].pose.linear()*=expSO3(dx.segment<3>(26+6*i));
    J.block<3,3>(26+6*i,26+6*i)=rightJacobian(dx.segment<3>(26+6*i));
  }
  covariance_=J*covariance_*J.transpose();covariance_=(covariance_+covariance_.transpose()).eval()*.5;
  return Correction{dx.head<23>(),covariance_.topLeftCorner<23,23>()};
}
std::optional<LinearObservation> CloneFilter::finishTrack(const std::vector<FeatureObservation>& track,const Camera& c){
  if(track.size()<2)return {};
  std::map<uint64_t,int> index;for(size_t i=0;i<clones_.size();++i)index[clones_[i].id]=i;
  auto first=index.find(track.front().clone);if(first==index.end())return {};
  if(track.front().uv.x()-track.front().uv.z()<1)return {};
  const Vec3 world=(clones_[first->second].pose*c.body_T_camera)*c.triangulate(track.front().uv);
  const int m=3*track.size(),n=covariance_.rows();
  Eigen::MatrixXd H=Eigen::MatrixXd::Zero(m,n),Hf(m,3);Eigen::VectorXd residual(m);
  for(size_t i=0;i<track.size();++i){
    auto it=index.find(track[i].clone);if(it==index.end())return {};
    const Pose& pose=clones_[it->second].pose;const Pose T=pose*c.body_T_camera;
    Vec3 p=T.inverse()*world;
    if(p.z()<.2||p.z()>100)return {};
    const Vec3 predicted=c.project(p);if((predicted-track[i].uv).norm()>5*c.pixel_sigma)return {};
    Eigen::Matrix<double,3,3> J;
    J<<c.fx/p.z(),0,-c.fx*p.x()/(p.z()*p.z()),0,c.fy/p.z(),-c.fy*p.y()/(p.z()*p.z()),c.fx/p.z(),0,-c.fx*(p.x()-c.baseline)/(p.z()*p.z());
    Hf.block<3,3>(3*i,0)=J*T.linear().transpose();
    H.block<3,3>(3*i,23+6*it->second)=-J*T.linear().transpose();
    H.block<3,3>(3*i,26+6*it->second)=J*c.body_T_camera.linear().transpose()*skew(pose.inverse()*world);
    residual.segment<3>(3*i)=track[i].uv-predicted;
  }
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(Hf,Eigen::ComputeFullU);
  if(svd.singularValues().minCoeff()<1e-6)return {};
  Eigen::MatrixXd A=svd.matrixU().rightCols(m-3);
  LinearObservation o;o.H=A.transpose()*H;o.residual=A.transpose()*residual;
  o.R=Eigen::MatrixXd::Identity(m-3,m-3)*c.pixel_sigma*c.pixel_sigma;return o;
}
std::optional<Correction> CloneFilter::stereo(const StereoFrame& frame,const State& s,const Camera& camera){
  clone(frame.id,s);
  std::set<uint64_t> present;for(const auto& p:frame.pixels){present.insert(p.feature);tracks_[p.feature].push_back({frame.id,p.uv});}
  std::vector<LinearObservation> observations;int rows=0;
  for(auto it=tracks_.begin();it!=tracks_.end();){
    const bool oldest=!clones_.empty()&&it->second.front().clone==clones_.front().id;
    if(!present.count(it->first)||(clones_.size()>=max_clones&&oldest)){
      auto observation=finishTrack(it->second,camera);
      if(observation){rows+=observation->H.rows();observations.push_back(std::move(*observation));}
      it=tracks_.erase(it);
    }else ++it;
  }
  std::optional<Correction> total;
  if(rows){LinearObservation batch;batch.H=Eigen::MatrixXd::Zero(rows,covariance_.rows());batch.residual.resize(rows);batch.variances=Eigen::VectorXd::Constant(rows,camera.pixel_sigma*camera.pixel_sigma);int offset=0;
    for(const auto& o:observations){batch.H.middleRows(offset,o.H.rows())=o.H;batch.residual.segment(offset,o.H.rows())=o.residual;offset+=o.H.rows();}
    total=update(batch);
  }

  while(clones_.size()>=max_clones)removeOldest();
  return total;
}
void CloneFilter::removeOldest(){
  if(clones_.empty())return;
  int n=covariance_.rows();Eigen::MatrixXd P(n-6,n-6);
  P.topLeftCorner<23,23>()=covariance_.topLeftCorner<23,23>();
  if(n>29){P.topRightCorner(23,n-29)=covariance_.topRightCorner(23,n-29);P.bottomLeftCorner(n-29,23)=P.topRightCorner(23,n-29).transpose();P.bottomRightCorner(n-29,n-29)=covariance_.bottomRightCorner(n-29,n-29);}
  covariance_=P;uint64_t id=clones_.front().id;clones_.erase(clones_.begin());
  for(auto it=tracks_.begin();it!=tracks_.end();) {if(!it->second.empty()&&it->second.front().clone==id)it=tracks_.erase(it);else ++it;}
}
}
