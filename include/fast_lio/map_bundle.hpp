#pragma once
#include "fast_lio/backend.hpp"
namespace fastlio {
struct MapKeyframe {
  uint64_t id = 0;
  double time = 0;
  Pose pose = Pose::Identity(); // map_T_imu
  Cloud::ConstPtr cloud; // imu frame
};
struct MapBundle {
  std::vector<MapKeyframe> keyframes;
  Cloud::ConstPtr map;
};
// Atomic manifest commit; generation files are immutable once committed.
void saveMapBundle(const std::string& directory, const std::vector<MapKeyframe>& frames);
MapBundle loadMapBundle(const std::string& directory, double voxel = .3);
}
