#pragma once
#include "fast_lio/types.hpp"
#include <mutex>
#include <optional>
namespace fastlio {
// Separate predictor. Corrections replay history but never republish old samples.
class Propagator {
 public:
  Noise noise;
  double max_gap=.1, acceleration_scale=1.;
  size_t max_samples=10000;
  std::optional<State> push(const Imu& imu);
  bool correct(const State& anchor);
  void reset();
  uint64_t gaps() const {std::lock_guard<std::mutex> lock(mutex_);return gaps_;}
  void configure(const Noise& n,double scale){std::lock_guard<std::mutex> lock(mutex_);noise=n;acceleration_scale=scale;}
  static bool integrate(State& s,const Imu& a,const Imu& b,const Noise& noise,double scale,double max_gap);
 private:
  mutable std::mutex mutex_;
  std::deque<Imu> history_;
  State state_;
  bool ready_=false;
  double last_published_=-1,anchor_time_=-1;
  uint64_t anchor_version_=0,gaps_=0;
};
}
