#pragma once
#include "fast_lio/types.hpp"
#include "fast_lio/degeneracy.hpp"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <memory>
#include <optional>
#include <functional>
namespace fastlio {
using Cloud=pcl::PointCloud<pcl::PointXYZI>;
struct BackendEvent {
  State state;
  std::optional<StereoFrame> stereo;
  std::optional<Camera> camera;
  std::optional<Fix> fix;
  std::optional<Wheel> wheel;
  Cloud::ConstPtr cloud;
  DegeneracyStatus degeneracy;
  uint64_t session=0;
};
struct BackendResult {
  uint64_t session=0;
  State local;
  Pose map_T_odom=Pose::Identity();
  Mat6 covariance=Mat6::Identity();
  std::vector<std::pair<double,Pose>> trajectory;
  Cloud::Ptr map;
  uint64_t loops=0,processed=0,dropped=0;
  uint64_t wheel_factors=0,gnss_factors=0,visual_factors=0;
  std::string error;
};
struct BackendConfig {
  bool enabled=true,loop_enabled=true,nonholonomic=true;
  Side side=Side::Backend;
  Camera camera;
  Pose imu_T_base=Pose::Identity();Vec3 antenna=Vec3::Zero();
  double key_distance=1.,key_angle=.2,key_seconds=1.,voxel=.3;
  double loop_distance=.2,icp_fitness=.15,loop_overlap=.5,loop_max_translation=10.,loop_max_correction=5.,loop_max_rotation=.6;
  int loop_separation=30,loop_interval=10,visual_track_length=8;
  size_t max_queue=256;
  std::string output_directory="/tmp/fastlio-output";
};
class Backend {
 public:
  explicit Backend(const BackendConfig& config,std::function<void(const BackendResult&)> callback={});
  ~Backend();
  bool submit(BackendEvent event);
  void reset(uint64_t session);
  BackendResult snapshot()const;
  bool save(std::string& error);
  void flush();
 private:
  struct Impl;std::unique_ptr<Impl> impl_;
};
}
