#include <gtest/gtest.h>
#include "fast_lio/backend.hpp"
#include <filesystem>
using namespace fastlio;
BackendEvent event(double t,double x){BackendEvent e;e.state.time=t;e.state.pose.translation().x()=x;e.state.covariance=Cov::Identity()*.01;Cloud::Ptr cloud(new Cloud);pcl::PointXYZI p;p.x=1;p.y=0;p.z=0;p.intensity=1;cloud->push_back(p);e.cloud=cloud;return e;}
TEST(Backend,GlobalOptimizationPreservesLocalPose){BackendConfig c;c.loop_enabled=false;c.key_distance=.1;Backend b(c);ASSERT_TRUE(b.submit(event(0,0)));ASSERT_TRUE(b.submit(event(1,1.2)));BackendEvent fix;fix.state.time=2;fix.state.pose.translation().x()=2.4;fix.state.covariance=Cov::Identity()*.01;fix.fix=Fix{2,Vec3(2,0,0),Mat3::Identity()*.01};ASSERT_TRUE(b.submit(fix));b.flush();auto result=b.snapshot();EXPECT_TRUE(result.error.empty())<<result.error;EXPECT_EQ(result.trajectory.size(),3);EXPECT_NEAR(result.local.pose.translation().x(),2.4,1e-12);EXPECT_LT(result.map_T_odom.translation().x(),-.1);}
TEST(Backend,FrontendModeDoesNotConsumeRawGnss){BackendConfig c;c.side=Side::Frontend;c.loop_enabled=false;Backend b(c);b.submit(event(0,0));BackendEvent e=event(1,1);e.fix=Fix{1,Vec3(0,0,0),Mat3::Identity()*.001};b.submit(e);b.flush();auto result=b.snapshot();EXPECT_TRUE(result.error.empty())<<result.error;EXPECT_NEAR(result.trajectory.back().second.translation().x(),1,1e-8);}
TEST(Backend,ResetRejectsOldSessionAndExports){BackendConfig c;c.output_directory="/tmp/fastlio-backend-test";c.loop_enabled=false;Backend b(c);b.submit(event(0,0));b.flush();b.reset(1);EXPECT_FALSE(b.submit(event(1,1)));auto e=event(2,2);e.session=1;ASSERT_TRUE(b.submit(e));b.flush();EXPECT_EQ(b.snapshot().trajectory.size(),1);std::string error;EXPECT_TRUE(b.save(error))<<error;EXPECT_TRUE(std::filesystem::exists(c.output_directory+"/map.pcd"));EXPECT_TRUE(std::filesystem::exists(c.output_directory+"/trajectory.tum"));}
TEST(Backend,CameraObservationsCreateTimestampedPoses){BackendConfig c;c.loop_enabled=false;Backend b(c);b.submit(event(0,0));Camera cam;cam.fx=cam.fy=300;cam.cx=320;cam.cy=240;cam.baseline=.2;for(int i=1;i<=4;++i){BackendEvent e;e.state.time=.1*i;e.state.pose.translation().x()=.1*i;e.state.covariance=Cov::Identity()*.01;e.camera=cam;StereoFrame f;f.time=e.state.time;f.id=i;f.pixels.push_back({1,cam.project(e.state.pose.inverse()*Vec3(0,0,5))});e.stereo=f;if(i==4)e.stereo->pixels.clear();b.submit(e);}b.flush();auto result=b.snapshot();EXPECT_TRUE(result.error.empty())<<result.error;EXPECT_EQ(result.trajectory.size(),5);EXPECT_NEAR(result.trajectory[1].first,.1,1e-8);}
TEST(Backend,WheelFactorsRespectMeasurementMode){
  for(auto side:{Side::Frontend,Side::Backend}){
    BackendConfig c;c.side=side;c.loop_enabled=false;c.key_distance=.1;
    Backend b(c);
    BackendEvent start;start.state.time=0;start.wheel=Wheel{0,1,.04};b.submit(start);
    b.submit(event(0,0));
    for(int i=1;i<=100;++i){BackendEvent w;w.state.time=i*.01;w.wheel=Wheel{i*.01,1,.04};b.submit(w);}
    b.submit(event(1,1));b.flush();auto result=b.snapshot();EXPECT_TRUE(result.error.empty())<<result.error;
    EXPECT_EQ(result.wheel_factors,side==Side::Backend?1:0);
  }
}
TEST(Backend,SameTimestampLidarCorrectionUpdatesGlobalTransform){
  BackendConfig c;c.loop_enabled=false;c.key_distance=.1;Backend b(c);b.submit(event(0,0));
  BackendEvent image;image.state.time=1;image.state.pose.translation().x()=1.2;
  image.state.covariance=Cov::Identity()*.01;image.stereo=StereoFrame{1,1,{}};b.submit(image);
  b.submit(event(1,1));b.flush();auto result=b.snapshot();EXPECT_TRUE(result.error.empty())<<result.error;
  EXPECT_NEAR(result.map_T_odom.translation().x(),0,1e-8);EXPECT_NEAR(result.trajectory.back().second.translation().x(),1,1e-8);
}
TEST(Backend,VerifiedLoopCorrectsMapWithoutChangingLocalOdom){
  BackendConfig c;c.key_distance=.1;c.loop_separation=2;c.loop_interval=1;
  c.loop_distance=.2;c.loop_overlap=.8;c.icp_fitness=.01;
  Backend b(c);
  Cloud world;uint32_t seed=42;
  auto random=[&](){seed=1664525*seed+1013904223;return double(seed%10000)/10000.;};
  for(int i=0;i<800;++i){pcl::PointXYZI p;p.x=20*random()-10;p.y=20*random()-10;p.z=4*random()-2;p.intensity=1;world.push_back(p);}
  for(int i=0;i<4;++i){double actual=i==3?0:double(i),local=i==3?.3:actual;
    BackendEvent e=event(i,local);Cloud::Ptr scan(new Cloud(world));
    for(auto& p:*scan)p.x-=actual;
    e.cloud=scan;ASSERT_TRUE(b.submit(e));
  }
  b.flush();auto result=b.snapshot();ASSERT_TRUE(result.error.empty())<<result.error;
  EXPECT_GT(result.loops,0);EXPECT_NEAR(result.local.pose.translation().x(),.3,1e-12);
  EXPECT_LT(result.trajectory.back().second.translation().x(),.15);
}

TEST(Backend,ImpossibleDescriptorAliasDoesNotHideNearbyLoop){
  BackendConfig config;config.key_distance=.1;config.loop_separation=2;config.loop_interval=1;
  config.loop_overlap=.8;config.icp_fitness=.01;Backend backend(config);
  Cloud pattern;uint32_t seed=42;
  auto random=[&](){seed=1664525*seed+1013904223;return double(seed%10000)/10000.;};
  for(int i=0;i<800;++i){pcl::PointXYZI p;p.x=20*random()-10;p.y=20*random()-10;p.z=4*random()-2;p.intensity=1;pattern.push_back(p);}
  const double positions[]={100,0,2,3,.3};
  for(int i=0;i<5;++i){BackendEvent e=event(i,positions[i]);Cloud::Ptr scan(new Cloud(pattern));
    if(i>0&&i<4)for(auto& p:*scan){p.x-=positions[i];p.z+=.02;}
    e.cloud=scan;ASSERT_TRUE(backend.submit(e));}
  backend.flush();auto result=backend.snapshot();ASSERT_TRUE(result.error.empty())<<result.error;
  EXPECT_GT(result.loops,0);
}

TEST(Backend,ScanContextYawAlignsCurrentCloudIntoHistoricalFrame){
  BackendConfig config;config.key_distance=.1;config.loop_separation=2;config.loop_interval=1;
  config.loop_overlap=.8;config.icp_fitness=.01;Backend backend(config);
  Cloud world;uint32_t seed=17;
  auto random=[&](){seed=1664525*seed+1013904223;return double(seed%10000)/10000.;};
  for(int i=0;i<800;++i){pcl::PointXYZI p;p.x=20*random()-10;p.y=20*random()-10;p.z=4*random()-2;p.intensity=1;world.push_back(p);}
  for(int i=0;i<4;++i){
    const bool returning=i==3;const double actual_x=returning?0.:double(i);
    const Mat3 rotation=expSO3(Vec3(0,0,returning?M_PI/2:0));
    BackendEvent e=event(i,returning?.3:actual_x);e.state.pose.linear()=rotation;
    Cloud::Ptr scan(new Cloud(world));
    for(auto& point:*scan){Vec3 p=rotation.transpose()*(Vec3(point.x,point.y,point.z)-Vec3(actual_x,0,0));point.x=p.x();point.y=p.y();point.z=p.z();}
    e.cloud=scan;ASSERT_TRUE(backend.submit(e));
  }
  backend.flush();auto result=backend.snapshot();ASSERT_TRUE(result.error.empty())<<result.error;
  EXPECT_GT(result.loops,0);EXPECT_NEAR(result.local.pose.translation().x(),.3,1e-12);
  EXPECT_LT(result.trajectory.back().second.translation().x(),.15);
}
