#pragma once
#include "fast_lio/backend.hpp"
namespace fastlio {
using Descriptor = Eigen::Matrix<double, 20, 60>;
Descriptor descriptor(const Cloud& cloud);
std::pair<double, int> descriptorDistance(const Descriptor& a, const Descriptor& b);
struct RegistrationConfig {
  double correspondence = 2., fitness = .15, overlap = .5, overlap_distance = .5;
  double minimum_geometry_ratio = 1e-4;
  int iterations = 60;
};
struct RegistrationResult {
  bool accepted = false;
  Pose pose = Pose::Identity();
  Mat6 covariance = Mat6::Identity(); // [map translation, body rotation]
  double fitness = INFINITY, overlap = 0;
};
RegistrationResult registerCloud(const Cloud::ConstPtr& source, const Cloud::ConstPtr& target,
                                 const Pose& guess, const RegistrationConfig& config);
Cloud::Ptr filteredCloud(const Cloud& cloud, double voxel);
}
