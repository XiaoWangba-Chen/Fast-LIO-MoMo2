#pragma once
#include "fast_lio/types.hpp"
#include "fast_lio/backend.hpp"
#include "fast_lio/clone_filter.hpp"
#include "fast_lio/propagator.hpp"
#include <rclcpp/rclcpp.hpp>
#include <functional>
#include <memory>
namespace fastlio {
class Runtime {
 public:
  explicit Runtime(rclcpp::Node::SharedPtr parameters);
  ~Runtime();
  CloneFilter filter;
  Propagator propagator;
  Degeneracy degeneracy;
  bool frontend()const;
  bool reset_requested()const;
  void request_reset();
  void reset(const Cov& P);
  void process(double end,const std::function<void(double)>& predict,const std::function<State()>& state,const std::function<void(const Correction&)>& apply,const std::function<Vec3()>& angular);
  void corrected(const State& state,const Cloud::ConstPtr& cloud);
  void noise(const Noise& noise,double acceleration_scale);
  void diagnostics(const DegeneracyStatus&,size_t lidar_queue,size_t imu_queue,double elapsed_ms);
  void finish();
 private:
  struct Impl;std::unique_ptr<Impl> impl_;
};
}
