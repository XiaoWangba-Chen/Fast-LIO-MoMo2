#include "fast_lio/localization.hpp"
#include <pcl/kdtree/kdtree_flann.h>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <algorithm>
namespace fastlio {
const char* phaseName(LocalizationPhase p) {
  switch (p) {
    case LocalizationPhase::Searching: return "searching";
    case LocalizationPhase::Confirming: return "confirming";
    case LocalizationPhase::Tracking: return "tracking";
    case LocalizationPhase::Lost: return "lost";
  }
  return "unknown";
}
namespace {
bool closePose(const Pose& a, const Pose& b, double translation, double rotation) {
  return (a.translation() - b.translation()).norm() <= translation && Eigen::AngleAxisd(
    a.linear().transpose() * b.linear()).angle() <= rotation;
}
}
struct Localization::Impl {
  LocalizationConfig config;
  MapBundle bundle;
  std::vector<Descriptor> descriptors;
  pcl::KdTreeFLANN<pcl::PointXYZI> tree;
  std::function<void(const LocalizationResult&)> callback;
  mutable std::mutex mutex;
  std::condition_variable changed, idle;
  std::thread worker;
  bool stopping = false, busy = false;
  uint64_t revision = 0, session = 0, dropped = 0;
  double last_submitted = -1;
  struct Job {
    State state;
    Cloud::ConstPtr cloud;
  };
  std::optional<Job> pending;
  struct Seed {
    Pose pose;
    double time;
  };
  std::optional<Seed> seed;
  LocalizationResult result;
  explicit Impl(const LocalizationConfig& c, std::function<void(const LocalizationResult&)> cb) : config(c),
    callback(std::move(cb)) {
    auto positive = [](double v) {
                      return std::isfinite(v) && v > 0;
                    };
    if (!positive(c.voxel) || !positive(c.descriptor_threshold) || c.descriptor_threshold > 1 ||
        !positive(c.submap_radius) || !positive(c.consistency_translation) ||
        !positive(c.consistency_rotation) || !positive(c.maximum_correction_translation) ||
        !positive(c.maximum_correction_rotation) || !positive(c.ambiguity_ratio) || c.ambiguity_ratio < 1 ||
        !positive(c.ambiguity_distance) || !positive(c.ambiguity_fitness_margin) || c.candidates < 1 ||
        c.confirmations < 1 || c.failures < 1 || !positive(c.registration.correspondence) ||
        !positive(c.registration.fitness) || !positive(c.registration.overlap) ||
        c.registration.overlap > 1 ||
        !positive(c.registration.overlap_distance) || !positive(c.registration.minimum_geometry_ratio) ||
        c.registration.minimum_geometry_ratio >= 1 ||
        c.registration.iterations < 1) {
      throw std::invalid_argument("Invalid localization parameters");
    }
    bundle = loadMapBundle(c.map_directory, c.voxel);
    tree.setInputCloud(bundle.map);
    for (const auto& f:bundle.keyframes) {
      descriptors.push_back(descriptor(*f.cloud));
    }
    worker = std::thread([this] {
        run();
      });
  }
  ~Impl() {
    {
      std::lock_guard<std::mutex> l(mutex);
      stopping = true;
      pending.reset();
      changed.notify_one();
    }
    worker.join();
  }
  Cloud::Ptr submap(const Vec3& center) {
    pcl::PointXYZI p;
    p.x = center.x();
    p.y = center.y();
    p.z = center.z();
    std::vector<int> ids;
    std::vector<float> ds;
    tree.radiusSearch(p, config.submap_radius, ids, ds);
    Cloud::Ptr out(new Cloud);
    out->reserve(ids.size());
    for (int id:ids) {
      out->push_back((*bundle.map)[id]);
    }
    return out;
  }
  RegistrationResult search(const Cloud::ConstPtr& cloud) {
    Descriptor d = descriptor(*cloud);
    struct Candidate {
      double distance;
      size_t index;
      int shift;
    };
    std::vector<Candidate> candidates;
    for (size_t i = 0; i < descriptors.size(); ++i) {
      auto match = descriptorDistance(d, descriptors[i]);
      if (match.first <= config.descriptor_threshold) {
        candidates.push_back({match.first, i, match.second});
      }
    }
    std::sort(candidates.begin(), candidates.end(), [](const auto& a, const auto& b) {
        return a.distance < b.distance;
      });
    if (candidates.size() > static_cast<size_t>(config.candidates)) {
      candidates.resize(config.candidates);
    }
    std::vector<RegistrationResult> matches;
    for (const auto& c:candidates) {
      const auto& f = bundle.keyframes[c.index];
      Pose yaw = Pose::Identity();
      yaw.linear() = Eigen::AngleAxisd(c.shift * 2 * M_PI / 60, Vec3::UnitZ()).toRotationMatrix();
      auto r = registerCloud(cloud, submap(f.pose.translation()), f.pose * yaw, config.registration);
      if (r.accepted) {
        matches.push_back(r);
      }
    }
    if (matches.empty()) {
      return {};
    }
    std::sort(matches.begin(), matches.end(), [](const auto& a, const auto& b) {
        return a.fitness < b.fitness;
      });
    for (size_t i = 1; i < matches.size();
         ++i) {
      if (!closePose(matches[0].pose, matches[i].pose, config.ambiguity_distance,
                     .5) &&
          matches[i].fitness <=
          std::max(config.ambiguity_ratio * matches[0].fitness,
                   matches[0].fitness + config.ambiguity_fitness_margin)) {
        return {};
      }
    }
    return matches[0];
  }
  void process(LocalizationResult& r, const Job& job, std::optional<Seed> manual) {
    r.local = job.state;
    r.valid = false;
    r.error.clear();
    auto cloud = filteredCloud(*job.cloud, config.voxel);
    RegistrationResult match;
    bool tracking = r.phase == LocalizationPhase::Tracking;
    if (manual) {
      if (manual->time != 0 && std::abs(manual->time - job.state.time) > .5) {
        r.error = "Initial pose is more than 0.5 s from the scan";
        return;
      }
      match = registerCloud(cloud, submap(manual->pose.translation()), manual->pose, config.registration);
    }
    else if (tracking || r.phase == LocalizationPhase::Confirming) {
      Pose predicted = r.map_T_odom * job.state.pose;
      match = registerCloud(cloud,
                            submap(predicted.translation()),
                            predicted,
                            config.registration);
      if (match.accepted &&
          !closePose(match.pose, predicted, config.maximum_correction_translation,
                     config.maximum_correction_rotation)) {
        match.accepted = false;
      }
    }
    else{
      match = search(cloud);
    }
    r.fitness = match.fitness;
    r.overlap = match.overlap;
    if (!match.accepted) {
      ++r.failures;
      r.confirmations = 0;
      if (tracking && r.failures < config.failures) {
        return;
      }
      r.phase =
        tracking?LocalizationPhase::Lost:(r.phase ==
          LocalizationPhase::Lost?LocalizationPhase::Lost:LocalizationPhase::
          Searching);
      return;
    }
    Pose correction = match.pose * job.state.pose.inverse();
    if (!tracking) {
      if (r.confirmations > 0 &&
          !closePose(correction, r.map_T_odom, config.consistency_translation,
                     config.consistency_rotation)) {
        r.confirmations = 0;
      }
      ++r.confirmations;
      r.phase = r.confirmations >=
                config.confirmations?LocalizationPhase::Tracking:LocalizationPhase::Confirming;
    }
    r.map_T_odom = correction;
    r.covariance = match.covariance;
    r.failures = 0;
    r.valid = r.phase == LocalizationPhase::Tracking;
  }
  void run() {
    LocalizationResult current;
    uint64_t current_revision = UINT64_MAX;
    for (;;) {
      Job job;
      std::optional<Seed> manual;
      uint64_t active_revision;
      {
        std::unique_lock<std::mutex> l(mutex);
        changed.wait(l, [this] {
            return stopping || pending.has_value();
          });
        if (stopping) {
          return;
        }
        job = std::move(*pending);
        pending.reset();
        busy = true;
        active_revision = revision;
        if (current_revision != revision) {
          current = LocalizationResult();
          current.session = session;
          current_revision = revision;
        }
        manual = seed;
        seed.reset();
      }
      try{
        process(current, job, manual);
      }
      catch (const std::exception& e) {
        current.valid = false;
        current.phase = LocalizationPhase::Lost;
        current.confirmations = 0;
        current.error = e.what();
      }
      // Serialize publication against reset/initialPose: stale work cannot publish.
      {
        std::lock_guard<std::mutex> l(mutex);
        if (active_revision == revision) {
          current.dropped = dropped;
          result = current;
          if (callback) {
            callback(result);
          }
        }
        busy = false;
        idle.notify_all();
      }
    }
  }
};
Localization::Localization(const LocalizationConfig& c,
                           std::function<void(const LocalizationResult&)> cb) : impl_(new Impl(c,
                                                                                               std::move(cb))) {
}
Localization::~Localization() = default;
void Localization::submit(const State& state, const Cloud::ConstPtr& cloud, uint64_t session) {
  if (!cloud || cloud->empty() || !std::isfinite(state.time) || state.time < 0 ||
      !state.pose.matrix().allFinite()) {
    return;
  }
  std::lock_guard<std::mutex> l(impl_->mutex);
  if (session != impl_->session || state.time <= impl_->last_submitted) {
    return;
  }
  impl_->last_submitted = state.time;
  if (impl_->pending) {
    ++impl_->dropped;
  }
  impl_->pending = Impl::Job{state, cloud};
  impl_->changed.notify_one();
}
void Localization::reset(uint64_t session) {
  std::lock_guard<std::mutex> l(impl_->mutex);
  ++impl_->revision;
  impl_->session = session;
  impl_->last_submitted = -1;
  impl_->pending.reset();
  impl_->seed.reset();
  impl_->result = LocalizationResult();
  impl_->result.session = session;
  impl_->idle.notify_all();
}
void Localization::initialPose(const Pose& p, double time) {
  if (!p.matrix().allFinite() || !std::isfinite(time) || time < 0 ||
      !p.linear().transpose().isApprox(p.linear().inverse(),
                                       1e-5) ||
      std::abs(p.linear().determinant() - 1) > 1e-5) {
    throw std::invalid_argument("Invalid initial pose");
  }
  std::lock_guard<std::mutex> l(impl_->mutex);
  ++impl_->revision;
  impl_->last_submitted = -1;
  impl_->pending.reset();
  impl_->seed = Impl::Seed{p, time};
  impl_->result = LocalizationResult();
  impl_->result.session = impl_->session;
  impl_->idle.notify_all();
}
LocalizationResult Localization::snapshot()const {
  std::lock_guard<std::mutex> l(impl_->mutex);
  return impl_->result;
}
Cloud::ConstPtr Localization::map()const {
  return impl_->bundle.map;
}
void Localization::flush() {
  std::unique_lock<std::mutex> l(impl_->mutex);
  impl_->idle.wait(l, [this] {
      return !impl_->busy && !impl_->pending;
    });
}
}
