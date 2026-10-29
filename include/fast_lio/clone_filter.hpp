#pragma once
#include "fast_lio/types.hpp"
#include "fast_lio/observations.hpp"
#include <map>
namespace fastlio {
struct Clone {uint64_t id;double time;Pose pose;};
struct Correction {Eigen::Matrix<double,23,1> delta;Cov covariance;};
class CloneFilter {
 public:
  size_t max_clones=10;
  void reset(const Cov& covariance);
  void predict(const Cov& transition,const Cov& covariance);
  void condition(const Cov& prior,const Cov& posterior,const Eigen::Matrix<double,23,1>& delta,const Cov& reset);
  void clone(uint64_t id,const State& state);
  std::optional<Correction> update(const LinearObservation& observation,double gate_per_dof=9.);
  std::optional<Correction> stereo(const StereoFrame& frame,const State& state,const Camera& camera);
  void removeOldest();
  void transportCurrent(const Cov& J);
  const Eigen::MatrixXd& covariance() const {return covariance_;}
  const std::vector<Clone>& clones()const{return clones_;}
 private:
  Eigen::MatrixXd covariance_=Cov::Identity();
  std::vector<Clone> clones_;
  struct FeatureObservation {uint64_t clone;Vec3 uv;};
  std::map<uint64_t,std::vector<FeatureObservation>> tracks_;
  std::optional<LinearObservation> finishTrack(const std::vector<FeatureObservation>& track,const Camera& camera);
};
}
