#include "fast_lio/map_bundle.hpp"
#include "fast_lio/registration.hpp"
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <chrono>
#include <set>
namespace fastlio {
namespace fs = std::filesystem;
namespace {
void checkPose(const Pose& p) {
  if (!p.matrix().allFinite() ||
      !p.linear().transpose().isApprox(p.linear().inverse(),
                                       1e-5) || std::abs(p.linear().determinant() - 1) > 1e-5) {
    throw std::runtime_error("Invalid map keyframe pose");
  }
}
}
void saveMapBundle(const std::string& directory, const std::vector<MapKeyframe>& frames) {
  if (frames.empty()) {
    throw std::runtime_error("No keyframes to save");
  }
  fs::create_directories(directory);
  std::string generation = "generation_" + std::to_string(
    std::chrono::steady_clock::now().time_since_epoch().count());
  fs::path root(directory), temporary = root / (generation + ".tmp"), destination = root / generation;
  fs::create_directory(temporary);
  fs::path manifest = root / (generation + ".manifest.tmp");
  try{
    std::ofstream out(manifest);
    if (!out) {
      throw std::runtime_error("Cannot write map manifest");
    }
    out << "FASTLIO_MAP 1\n" << generation << "\n" << frames.size() << "\n" << std::setprecision(17);
    Cloud map;
    std::set<uint64_t> ids;
    for (const auto& f:frames) {
      checkPose(f.pose);
      if (!std::isfinite(f.time) || !f.cloud || f.cloud->empty() ||
          !ids.insert(f.id).second) {
        throw std::runtime_error("Invalid map keyframe");
      }
      for (const auto& p:
           *f.cloud) {
        if (!std::isfinite(p.x) || !std::isfinite(p.y) ||
            !std::isfinite(p.z)) {
          throw std::runtime_error("Non-finite map point");
        }
      }
      Eigen::Quaterniond q(f.pose.linear());
      out << f.id << " " << f.time << " " << f.pose.translation().transpose() << " " << q.x() << " " <<
        q.y() << " " << q.z() << " " << q.w() << " " << f.cloud->size() << "\n";
      if (pcl::io::savePCDFileBinary((temporary / ("keyframe_" + std::to_string(f.id) + ".pcd")).string(),
                                     *f.cloud) < 0) {
        throw std::runtime_error("Cannot write keyframe PCD");
      }
      Cloud transformed;
      pcl::transformPointCloud(*f.cloud, transformed, f.pose.matrix().cast<float>());
      map += transformed;
    }
    if (pcl::io::savePCDFileBinary((temporary / "map.pcd").string(), map) < 0) {
      throw std::runtime_error(
        "Cannot write map PCD");
    }
    out.close();
    if (!out) {
      throw std::runtime_error("Cannot finish map manifest");
    }
    fs::rename(temporary, destination);
    fs::rename(manifest, root / "manifest.txt");
  }
  catch (...) {
    fs::remove_all(temporary);
    fs::remove(manifest);
    throw;
  }
}
MapBundle loadMapBundle(const std::string& directory, double voxel) {
  if (!std::isfinite(voxel) || voxel <= 0) {
    throw std::invalid_argument(
      "Map voxel size must be finite and positive");
  }
  fs::path root(directory);
  std::ifstream in(root / "manifest.txt");
  if (!in) {
    throw std::runtime_error("Missing manifest.txt: localization requires a new keyframe map bundle");
  }
  std::string magic, generation;
  int version = 0;
  size_t count = 0;
  if (!(in >> magic >> version) || magic != "FASTLIO_MAP" || version != 1) {
    throw std::runtime_error(
      "Unsupported map bundle version");
  }
  if (!(in >> generation >> count) ||
      generation.size() <= 11 || generation.rfind("generation_",
                       0) != 0 ||
      generation.find_first_not_of("0123456789",
                                   11) != std::string::npos || count == 0 ||
      count > 1000000) {
    throw std::runtime_error(
      "Invalid map bundle manifest");
  }
  MapBundle bundle;
  Cloud::Ptr map(new Cloud);
  std::set<uint64_t> ids;
  size_t total_points = 0;
  for (size_t i = 0; i < count; ++i) {
    MapKeyframe f;
    Vec3 t;
    double x, y, z, w;
    size_t points = 0;
    if (!(in >> f.id >> f.time >> t.x() >> t.y() >> t.z() >> x >> y >> z >> w >> points) ||
        !std::isfinite(f.time) || !t.allFinite() || !ids.insert(f.id).second) {
      throw std::runtime_error(
        "Malformed map keyframe metadata");
    }
    Eigen::Quaterniond q(w, x, y, z);
    if (!q.coeffs().allFinite() || std::abs(q.norm() - 1) > 1e-5) {
      throw std::runtime_error(
        "Invalid map quaternion");
    }
    f.pose.translation() = t;
    f.pose.linear() = q.normalized().toRotationMatrix();
    checkPose(f.pose);
    Cloud::Ptr cloud(new Cloud);
    if (pcl::io::loadPCDFile((root / generation / ("keyframe_" + std::to_string(f.id) + ".pcd")).string(),
                             *cloud) < 0 || cloud->size() != points) {
      throw std::runtime_error(
        "Missing or incomplete keyframe PCD");
    }
    for (const auto& p:
         *cloud) {
      if (!std::isfinite(p.x) || !std::isfinite(p.y) ||
          !std::isfinite(p.z)) {
        throw std::runtime_error(
          "Non-finite map point");
      }
    }
    total_points += cloud->size();
    f.cloud = filteredCloud(*cloud, voxel);
    if (f.cloud->size() < 30) {
      throw std::runtime_error("Map keyframe contains fewer than 30 usable points");
    }
    Cloud transformed;
    pcl::transformPointCloud(*f.cloud, transformed, f.pose.matrix().cast<float>());
    *map += transformed;
    bundle.keyframes.push_back(std::move(f));
  }
  std::string extra;
  if (in >> extra) {
    throw std::runtime_error("Unexpected map manifest data");
  }
  Cloud stored;
  if (pcl::io::loadPCDFile((root / generation / "map.pcd").string(),
                           stored) < 0 || stored.size() != total_points) {
    throw std::runtime_error(
      "Missing or incomplete global map PCD");
  }
  for (const auto& p:
       stored) {
    if (!std::isfinite(p.x) || !std::isfinite(p.y) ||
        !std::isfinite(p.z)) {
      throw std::runtime_error(
        "Non-finite global map point");
    }
  }
  bundle.map = filteredCloud(*map, voxel);
  return bundle;
}
}
