#include "fast_lio/registration.hpp"
#include <pcl/registration/icp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <Eigen/Eigenvalues>
namespace fastlio {
Descriptor descriptor(const Cloud& cloud) {
  Descriptor d = Descriptor::Zero();
  for (const auto& p:cloud) {
    double range = std::hypot(p.x, p.y);
    if (range >= 80 || range < 1) {
      continue;
    }
    int ring = std::min(19, int(range / 4));
    int sector = std::min(59, int((std::atan2(p.y, p.x) + M_PI) * 60 / (2 * M_PI)));
    d(ring, sector) = std::max(d(ring, sector), double(p.z + 5));
  }
  return d;
}
std::pair<double, int> descriptorDistance(const Descriptor& a, const Descriptor& b) {
  double best = 1;
  int best_shift = 0;
  for (int shift = 0; shift < 60; ++shift) {
    double sum = 0;
    int count = 0;
    for (int j = 0; j < 60; ++j) {
      auto x = a.col(j);
      auto y = b.col((j + shift) % 60);
      if (x.norm() < 1e-6 || y.norm() < 1e-6) {
        continue;
      }
      sum += x.dot(y) / (x.norm() * y.norm());
      ++count;
    }
    double distance = count > 10?1 - sum / count:1;
    if (distance < best) {
      best = distance;
      best_shift = shift;
    }
  }
  return {best, best_shift};
}
Cloud::Ptr filteredCloud(const Cloud& cloud, double voxel) {
  Cloud::Ptr finite(new Cloud);
  for (const auto& p:
       cloud) {
    if (std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z)) {
      finite->push_back(p);
    }
  }
  if (finite->empty()) {
    return finite;
  }
  pcl::VoxelGrid<pcl::PointXYZI> filter;
  filter.setLeafSize(voxel, voxel, voxel);
  filter.setInputCloud(finite);
  Cloud::Ptr out(new Cloud);
  filter.filter(*out);
  return out;
}
RegistrationResult registerCloud(const Cloud::ConstPtr& source,
                                 const Cloud::ConstPtr& target,
                                 const Pose& guess,
                                 const RegistrationConfig& c) {
  RegistrationResult r;
  if (!source || !target || source->size() < 30 || target->size() < 30 ||
      !guess.matrix().allFinite()) {
    return r;
  }
  pcl::IterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> icp;
  icp.setInputSource(source);
  icp.setInputTarget(target);
  icp.setMaximumIterations(c.iterations);
  icp.setMaxCorrespondenceDistance(c.correspondence);
  icp.setTransformationEpsilon(1e-6);
  Cloud aligned;
  icp.align(aligned, guess.matrix().cast<float>());
  if (!icp.hasConverged()) {
    return r;
  }
  r.pose.matrix() = icp.getFinalTransformation().cast<double>();
  if (!r.pose.matrix().allFinite() || std::abs(r.pose.linear().determinant() - 1) > .01) {
    return r;
  }
  pcl::KdTreeFLANN<pcl::PointXYZI> tree;
  tree.setInputCloud(target);
  std::vector<int> ids(1);
  std::vector<float> distances(1);
  // A point-to-point Hessian understates plane degeneracy. Target-neighborhood
  // normals provide a point-to-plane observability gate and covariance estimate.
  Mat6 information = Mat6::Zero();
  double squared_error = 0, plane_error = 0;
  size_t matches = 0, normals = 0;
  for (size_t i = 0; i < aligned.size(); ++i) {
    const auto& p = aligned[i];
    if (!tree.nearestKSearch(p, 1, ids,
                             distances) || distances[0] > c.overlap_distance * c.overlap_distance) {
      continue;
    }
    ++matches;
    squared_error += distances[0];
    std::vector<int> neighbors;
    std::vector<float> ds;
    if (tree.nearestKSearch(p, 8, neighbors, ds) < 8 || ds.back() > 4.) {
      continue;
    }
    Vec3 center = Vec3::Zero();
    for (int id:neighbors) {
      center += (*target)[id].getVector3fMap().cast<double>();
    }
    center /= neighbors.size();
    Mat3 scatter = Mat3::Zero();
    for (int id:neighbors) {
      Vec3 v = (*target)[id].getVector3fMap().cast<double>() - center;
      scatter += v * v.transpose();
    }
    Eigen::SelfAdjointEigenSolver<Mat3> eig(scatter);
    if (eig.info() != Eigen::Success || eig.eigenvalues()[1] < 1e-6 ||
        eig.eigenvalues()[0] > .3 * eig.eigenvalues()[1]) {
      continue;
    }
    Vec3 n = eig.eigenvectors().col(0), body = (*source)[i].getVector3fMap().cast<double>();
    Eigen::Matrix<double, 1, 6> J;
    J.head<3>() = n.transpose();
    J.tail<3>() = -n.transpose() * r.pose.linear() * skew(body);
    information += J.transpose() * J;
    double residual = n.dot(p.getVector3fMap().cast<double>() - center);
    plane_error += residual * residual;
    ++normals;
  }
  r.overlap = double(matches) / source->size();
  r.fitness = matches?squared_error / matches:INFINITY;
  if (r.overlap < c.overlap || r.fitness > c.fitness || normals < 30) {
    return r;
  }
  // Normalize angular columns to metres using a characteristic scene radius.
  Mat6 scale = Mat6::Identity();
  scale.bottomRightCorner<3, 3>() *= .1;
  Mat6 normalized = scale * information * scale;
  Eigen::SelfAdjointEigenSolver<Mat6> eig(normalized);
  if (eig.info() != Eigen::Success ||
      eig.eigenvalues().minCoeff() < c.minimum_geometry_ratio * eig.eigenvalues().maxCoeff()) {
    return r;
  }
  r.covariance = std::max(.0025, plane_error / std::max<size_t>(1, normals - 6)) * information.inverse();
  r.accepted = r.covariance.allFinite();
  return r;
}
}
