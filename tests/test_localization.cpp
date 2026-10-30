#include <gtest/gtest.h>
#include "fast_lio/localization.hpp"
#include <pcl/common/transforms.h>
#include <filesystem>
#include <fstream>
#include <atomic>
#include <unistd.h>
using namespace fastlio;
namespace {
struct Fixture : testing::Test {
  std::string directory;
  Cloud::Ptr scene{new Cloud};
  Pose origin = Pose::Identity();
  void SetUp()override {
    static std::atomic<int> serial{0};
    directory = "/tmp/fastlio-localization-" + std::to_string(getpid()) + "-" + std::to_string(serial++);
    // Three asymmetrically sized surfaces plus a sloped wall, all observable.
    auto add = [&](double x, double y, double z) {
                 pcl::PointXYZI p;
                 p.x = x;
                 p.y = y;
                 p.z = z;
                 p.intensity = 1;
                 scene->push_back(p);
               };
    for (int i = 0; i < 65; ++i) {
      for (int j = 0; j < 45; ++j) {
        double a = i * .16, b = j * .16;
        add(a - 4, b - 3, -1.5);
        add(-4, a - 4, b - 1.5);
        add(a - 4, 5, b - 1.5);
        add(7 + .1 * b, a - 4, b - 1.5);
      }
    }
    origin.translation() = Vec3(25, -10, 2);
    origin.linear() = expSO3(Vec3(0, 0, .3));
    saveMapBundle(directory, {{7, 1., origin, scene}});
  }
  void TearDown()override {
    std::filesystem::remove_all(directory);
  }
  LocalizationConfig config() {
    LocalizationConfig c;
    c.map_directory = directory;
    c.voxel = .15;
    c.registration.fitness = .03;
    return c;
  }
  Cloud::ConstPtr scan(const Pose& relative) {
    Cloud::Ptr out(new Cloud);
    pcl::transformPointCloud(*scene, *out, relative.inverse().matrix().cast<float>());
    return out;
  }
  void send(Localization& l, double time, const Pose& relative = Pose::Identity(), uint64_t session = 0) {
    State s;
    s.time = time;
    s.pose = relative;
    l.submit(s, scan(relative), session);
    l.flush();
  }
};
TEST_F(Fixture, BundleRoundTripAndMissingFile) {
  auto b = loadMapBundle(directory, .15);
  ASSERT_EQ(b.keyframes.size(), 1);
  EXPECT_EQ(b.keyframes[0].id, 7);
  EXPECT_TRUE(b.keyframes[0].pose.matrix().isApprox(origin.matrix(), 1e-8));
  EXPECT_GT(b.map->size(), 100);
  std::ifstream in(directory + "/manifest.txt");
  std::string magic, generation;
  int version;
  in >> magic >> version >> generation;
  std::filesystem::remove(directory + "/" + generation + "/keyframe_7.pcd");
  EXPECT_THROW(loadMapBundle(directory), std::runtime_error);
}
TEST_F(Fixture, RejectsBadVersionAndIncompleteMetadata) {
  std::ofstream(directory + "/manifest.txt") << "FASTLIO_MAP 99\n";
  EXPECT_THROW(loadMapBundle(directory), std::runtime_error);
  std::ofstream(directory + "/manifest.txt") << "FASTLIO_MAP 1\ngeneration_123\n1\n";
  EXPECT_THROW(loadMapBundle(directory), std::runtime_error);
}
TEST_F(Fixture, FailedSavePreservesPreviousManifest) {
  EXPECT_THROW(saveMapBundle(directory, {{1, 1, origin, {}}}), std::runtime_error);
  auto b = loadMapBundle(directory, .15);
  ASSERT_EQ(b.keyframes.size(), 1);
  EXPECT_EQ(b.keyframes[0].id, 7);
}
TEST_F(Fixture, GlobalSearchConfirmsAndTracksWithoutChangingLocalPose) {
  Localization l(config());
  for (int i = 1; i <= 3; ++i) {
    send(l, i);
    EXPECT_EQ(l.snapshot().valid, i == 3);
  }
  auto r = l.snapshot();
  EXPECT_EQ(r.phase, LocalizationPhase::Tracking);
  EXPECT_TRUE(r.map_T_odom.matrix().isApprox(origin.matrix(), .03));
  Pose motion = Pose::Identity();
  motion.translation() = Vec3(.5, .1, 0);
  send(l, 4, motion);
  r = l.snapshot();
  ASSERT_TRUE(r.valid);
  EXPECT_TRUE(r.local.pose.matrix().isApprox(motion.matrix()));
  EXPECT_LT((r.map_T_odom.translation() - origin.translation()).norm(), .1);
  EXPECT_GT(r.covariance.diagonal().minCoeff(), 0);
}
TEST_F(Fixture, GlobalSearchHandlesNinetyDegreeYaw) {
  Localization l(config());
  Pose yaw = Pose::Identity();
  yaw.linear() = expSO3(Vec3(0, 0, M_PI / 2));
  for (int i = 1; i <= 3; ++i) {
    send(l, i, yaw);
  }
  auto r = l.snapshot();
  EXPECT_TRUE(r.valid) << phaseName(r.phase) << " fitness=" << r.fitness;
  EXPECT_LT(Eigen::AngleAxisd(r.map_T_odom.linear().transpose() * origin.linear()).angle(), .05);
}
TEST_F(Fixture, LostRecoversAndOldSessionsAreRejected) {
  auto c = config();
  c.failures = 2;
  Localization l(c);
  for (int i = 1; i <= 3; ++i) {
    send(l, i);
  }
  ASSERT_TRUE(l.snapshot().valid);
  Cloud::Ptr invalid(new Cloud);
  for (int i = 0; i < 50; ++i) {
    pcl::PointXYZI p;
    p.x = i;
    p.y = 0;
    p.z = 0;
    invalid->push_back(p);
  }
  for (int i = 4; i <= 5; ++i) {
    State s;
    s.time = i;
    l.submit(s, invalid, 0);
    l.flush();
    EXPECT_FALSE(l.snapshot().valid);
  }
  EXPECT_EQ(l.snapshot().phase, LocalizationPhase::Lost);
  for (int i = 6; i <= 8; ++i) {
    send(l, i);
  }
  EXPECT_TRUE(l.snapshot().valid);
  l.reset(1);
  send(l, 9);
  EXPECT_FALSE(l.snapshot().valid);
  EXPECT_LT(l.snapshot().local.time, 0);
  for (int i = 10; i <= 12; ++i) {
    send(l, i, Pose::Identity(), 1);
  }
  EXPECT_TRUE(l.snapshot().valid);
}
TEST_F(Fixture, ManualSeedMustBeVerifiedAndResetInvalidatesInflightWork) {
  Localization l(config());
  l.initialPose(origin);
  for (int i = 1; i <= 3; ++i) {
    send(l, i);
  }
  ASSERT_TRUE(l.snapshot().valid);
  l.initialPose(origin, 1.);
  send(l, 10);
  EXPECT_FALSE(l.snapshot().valid);
  EXPECT_FALSE(l.snapshot().error.empty());
  State s;
  s.time = 11;
  l.submit(s, scene, 0);
  l.reset(2);
  l.flush();
  EXPECT_EQ(l.snapshot().session, 2);
  EXPECT_FALSE(l.snapshot().valid);
  EXPECT_LT(l.snapshot().local.time, 0);
}
TEST_F(Fixture, PlanarGeometryCannotEstablishLocalization) {
  Cloud::Ptr plane(new Cloud);
  for (const auto& p:*scene) {
    if (std::abs(p.z + 1.5) < .001) {
      plane->push_back(p);
    }
  }
  auto r = registerCloud(plane, plane, Pose::Identity(), config().registration);
  EXPECT_FALSE(r.accepted);
}
TEST_F(Fixture, AmbiguousDistantPlacesAreRejected) {
  Pose other = origin;
  other.translation().x() += 200;
  saveMapBundle(directory, {{7, 1, origin, scene}, {8, 2, other, scene}});
  Localization l(config());
  for (int i = 1; i <= 4; ++i) {
    send(l, i);
  }
  EXPECT_FALSE(l.snapshot().valid);
  EXPECT_EQ(l.snapshot().phase, LocalizationPhase::Searching);
  l.initialPose(origin);
  for (int i = 5; i <= 7; ++i) {
    send(l, i);
  }
  EXPECT_TRUE(l.snapshot().valid);
}
TEST_F(Fixture, InvalidConfigurationRejected) {
  auto c = config();
  c.candidates = 0;
  EXPECT_THROW(Localization l(c), std::invalid_argument);
}
}
