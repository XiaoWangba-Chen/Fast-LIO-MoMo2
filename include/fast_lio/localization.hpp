#pragma once
#include "fast_lio/map_bundle.hpp"
#include "fast_lio/registration.hpp"
namespace fastlio {
enum class LocalizationPhase {
  Searching, Confirming, Tracking, Lost
};
const char* phaseName(LocalizationPhase phase);
struct LocalizationConfig {
  std::string map_directory;
  double voxel = .3, descriptor_threshold = .2, submap_radius = 60.;
  double consistency_translation = 1., consistency_rotation = .25;
  double maximum_correction_translation = 2., maximum_correction_rotation = .35;
  double ambiguity_ratio = 1.2, ambiguity_distance = 3., ambiguity_fitness_margin = .01;
  int candidates = 5, confirmations = 3, failures = 5;
  RegistrationConfig registration;
};
struct LocalizationResult {
  LocalizationPhase phase = LocalizationPhase::Searching;
  uint64_t session = 0, dropped = 0;
  State local;
  Pose map_T_odom = Pose::Identity();
  Mat6 covariance = Mat6::Identity();
  double fitness = INFINITY, overlap = 0;
  int confirmations = 0, failures = 0;
  bool valid = false;
  std::string error;
};
class Localization {
 public:
  // Callback is serialized with reset/initialPose and must not reenter this object.
  explicit Localization(const LocalizationConfig& config,
                        std::function<void(const LocalizationResult&)> callback = {});
  ~Localization();
  void submit(const State& local, const Cloud::ConstPtr& cloud, uint64_t session);
  void reset(uint64_t session);
  // Pose is map_T_imu, at the next scan (zero time) or within 0.5 s of it.
  void initialPose(const Pose& pose, double time = 0);
  LocalizationResult snapshot()const;
  Cloud::ConstPtr map()const;
  void flush();
 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
}
