#include <gtest/gtest.h>
#include "fast_lio/degeneracy.hpp"
#include "fast_lio/propagator.hpp"
#include "fast_lio/clone_filter.hpp"
#include "fast_lio/stereo.hpp"
#include "fast_lio/scan_time.hpp"
#include <Eigen/Eigenvalues>
#include <opencv2/imgproc.hpp>
using namespace fastlio;
TEST(Routing,RejectsAmbiguousMode){EXPECT_EQ(parseSide("frontend"),Side::Frontend);EXPECT_EQ(parseSide("backend"),Side::Backend);EXPECT_THROW(parseSide("both"),std::invalid_argument);}
TEST(Degeneracy,FindsPlanarNullspaceAndHysteresis){
  Degeneracy d;d.minimum_points=5;Eigen::MatrixXd H=Eigen::MatrixXd::Zero(100,6);
  for(int i=0;i<100;++i){H(i,2)=1;H(i,3)=10*std::sin(i);H(i,4)=10*std::cos(i);}
  auto s=d.analyze(H,.01,100,.01,true);EXPECT_EQ(s.weak_dimensions,3);EXPECT_FALSE(s.degraded);
  EXPECT_TRUE(d.analyze(H,.01,100,.01,true).degraded);
  Eigen::MatrixXd full=Eigen::MatrixXd::Random(100,6);full.rightCols(3)*=10;
  for(int i=0;i<4;++i)EXPECT_TRUE(d.analyze(full,.01,100,.01,true).degraded);
  EXPECT_FALSE(d.analyze(full,.01,100,.01,true).degraded);
}
TEST(Degeneracy,CompensationNeverAddsInformation){
  Degeneracy d;Eigen::MatrixXd H=Eigen::MatrixXd::Random(200,13);H.col(0)*=.0001;
  auto s=d.analyze(H,.01,200,.01,true);auto out=d.compensate(H,s);
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eig(H.transpose()*H-out.transpose()*out);
  EXPECT_GE(eig.eigenvalues().minCoeff(),-1e-8);Mat6 scale=Mat6::Identity();scale.bottomRightCorner<3,3>()/=d.rotation_scale;EXPECT_LT((out.leftCols(6)*scale*s.weak_projector).norm(),1e-7);
}
TEST(Degeneracy,SeparatesInsufficientAndResidual){Degeneracy d;EXPECT_EQ(d.analyze(Eigen::MatrixXd::Zero(0,6),.01,100,1,true).reason,DegeneracyStatus::Insufficient);EXPECT_EQ(d.analyze(Eigen::MatrixXd::Random(100,6),.01,100,1,true).reason,DegeneracyStatus::Residual);}
TEST(Propagation,StationaryAndReplayAgree){
  Propagator p;State s;s.time=0;s.version=1;
  p.push({0,Vec3(0,0,9.81),Vec3::Zero()});ASSERT_TRUE(p.correct(s));State last;
  for(int i=1;i<=200;++i){auto out=p.push({i*.005,Vec3(0,0,9.81),Vec3::Zero()});ASSERT_TRUE(out);last=*out;}
  EXPECT_LT(last.pose.translation().norm(),1e-10);EXPECT_LT(last.velocity.norm(),1e-10);
  s.time=.5;s.version=2;s.velocity=Vec3(1,0,0);s.pose.translation()=Vec3(2,0,0);
  ASSERT_TRUE(p.correct(s));auto out=p.push({1.005,Vec3(0,0,9.81),Vec3::Zero()});ASSERT_TRUE(out);EXPECT_NEAR(out->pose.translation().x(),2.505,1e-8);
  EXPECT_FALSE(p.push({1.005,Vec3(0,0,9.81),Vec3::Zero()}));s.version=1;EXPECT_FALSE(p.correct(s));
}
TEST(Propagation,GapStopsUntilFreshAnchor){Propagator p;State s;s.time=0;p.push({0,Vec3(0,0,9.81),Vec3::Zero()});p.correct(s);EXPECT_FALSE(p.push({.2,Vec3(0,0,9.81),Vec3::Zero()}));EXPECT_EQ(p.gaps(),1);EXPECT_FALSE(p.push({.205,Vec3(0,0,9.81),Vec3::Zero()}));s.time=.2;s.version=2;ASSERT_TRUE(p.correct(s));EXPECT_TRUE(p.push({.21,Vec3(0,0,9.81),Vec3::Zero()}));}
TEST(Propagation,ConstantAccelerationAndRotation){State s;s.time=0;for(int i=1;i<=200;++i){Imu a{(i-1)*.005,Vec3(1,0,9.81),Vec3::Zero()},b{i*.005,a.acceleration,a.angular_velocity};ASSERT_TRUE(Propagator::integrate(s,a,b,Noise(),1,.1));}EXPECT_NEAR(s.pose.translation().x(),.5,1e-8);EXPECT_NEAR(s.velocity.x(),1,1e-8);Eigen::SelfAdjointEigenSolver<Cov> eig(s.covariance);EXPECT_GT(eig.eigenvalues().minCoeff(),0);}
TEST(Geodesy,OriginAndEastMotion){Geodesy g;EXPECT_LT(g.convert(0,0,0).norm(),1e-9);Vec3 p=g.convert(0,.00001,0);EXPECT_NEAR(p.x(),1.1131949,1e-5);EXPECT_NEAR(p.y(),0,1e-8);EXPECT_THROW(g.convert(91,0,0),std::invalid_argument);}
TEST(Geodesy,AlignmentWaitsForObservableHeading){GnssAlignment a;State s;Fix f;EXPECT_FALSE(a.align(f,s,Vec3::Zero()));f.position=Vec3(10,0,0);s.pose.translation()=Vec3(0,10,0);auto out=a.align(f,s,Vec3::Zero());ASSERT_TRUE(out);EXPECT_NEAR(out->position.y(),10,1e-8);EXPECT_NEAR(a.yaw,M_PI/2,1e-8);}
State perturb(State s,int i,double epsilon){if(i<3)s.pose.translation()[i]+=epsilon;else if(i<6)s.pose.linear()*=expSO3(Vec3::Unit(i-3)*epsilon);else if(i>=12&&i<15)s.velocity[i-12]+=epsilon;else if(i>=15&&i<18)s.bg[i-15]+=epsilon;return s;}
TEST(Observations,WheelJacobianWithLeverArm){
  WheelModel model;model.imu_T_base.translation()=Vec3(.3,.2,.1);model.imu_T_base.linear()=expSO3(Vec3(.1,.2,.3));State s;s.pose.linear()=expSO3(Vec3(.2,-.1,.6));s.velocity=Vec3(2,.2,.1);s.bg=Vec3(.1,.2,.3);Wheel w{0,1,.1};Vec3 omega(.2,.1,.4);auto o=model.linearize(s,w,omega);
  for(int i=0;i<23;++i){auto plus=model.linearize(perturb(s,i,1e-6),w,omega);auto minus=model.linearize(perturb(s,i,-1e-6),w,omega);EXPECT_LT((o.H.col(i)+(plus.residual-minus.residual)/(2e-6)).norm(),1e-7)<<i;}
}
TEST(Observations,GnssJacobianWithLeverArm){State s;s.pose.linear()=expSO3(Vec3(.2,.3,.4));Vec3 lever(.2,.4,1);Fix f;auto o=positionObservation(s,f,lever);for(int i=0;i<23;++i){auto plus=positionObservation(perturb(s,i,1e-6),f,lever),minus=positionObservation(perturb(s,i,-1e-6),f,lever);EXPECT_LT((o.H.col(i)+(plus.residual-minus.residual)/(2e-6)).norm(),1e-7)<<i;}}
TEST(CloneFilter,CrossCovarianceAndMarginalization){CloneFilter f;State s;s.covariance=Cov::Identity();f.reset(s.covariance);f.clone(1,s);ASSERT_EQ(f.covariance().rows(),29);EXPECT_TRUE((f.covariance().topRightCorner<6,6>().isApprox(Mat6::Identity())));Cov F=Cov::Identity();F(0,12)=.1;Cov P=F*s.covariance*F.transpose();f.predict(F,P);EXPECT_NEAR(f.covariance()(12,23),0,1e-8);s.pose.translation().x()=1;f.clone(2,s);Eigen::MatrixXd expected=f.covariance();f.removeOldest();ASSERT_EQ(f.covariance().rows(),29);EXPECT_TRUE((f.covariance().bottomRightCorner<6,6>().isApprox(expected.bottomRightCorner<6,6>())));}
TEST(CloneFilter,VelocityUpdateUsesFullStateAndRejectsSlip){CloneFilter f;State s;f.reset(Cov::Identity());WheelModel model;auto o=model.linearize(s,Wheel{0,1,.01},Vec3::Zero());auto update=f.update(o);ASSERT_TRUE(update);EXPECT_GT(update->delta[12],.98);EXPECT_LT(update->covariance(12,12),.011);o.residual[0]=100;EXPECT_FALSE(f.update(o));}
TEST(CloneFilter,CompletedTrackConsumedOnceAndBounded){CloneFilter f;f.max_clones=3;State s;f.reset(s.covariance);Camera c;c.fx=c.fy=300;c.cx=320;c.cy=240;c.baseline=.2;Vec3 point(0,0,5);for(int i=0;i<10;++i){s.time=i*.1;s.pose.translation().x()=i*.1;StereoFrame frame;frame.id=i;frame.time=s.time;frame.pixels.push_back({1,c.project(s.pose.inverse()*point)});auto update=f.stereo(frame,s,c);if(update)EXPECT_LT(update->delta.norm(),1e-6);EXPECT_LE(f.clones().size(),2);Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eig(f.covariance());EXPECT_GE(eig.eigenvalues().minCoeff(),-1e-8);}}
TEST(CloneFilter,CompressedUpdateMatchesDense){CloneFilter f;f.reset(Cov::Identity());LinearObservation o;o.H=Eigen::MatrixXd::Random(70,23);o.residual=Eigen::VectorXd::Random(70)*.01;o.R=Eigen::MatrixXd::Identity(70,70)*.5;Eigen::MatrixXd K=o.H.transpose()*(o.H*o.H.transpose()+o.R).inverse();auto update=f.update(o);ASSERT_TRUE(update);EXPECT_LT((update->delta-K*o.residual).norm(),1e-8);}
TEST(Stereo,BlankFramesFailGracefully){StereoTracker tracker;cv::Mat blank(240,320,CV_8UC1,cv::Scalar(0));auto f=tracker.track(0,blank,blank);EXPECT_TRUE(f.pixels.empty());EXPECT_THROW(tracker.track(0,cv::Mat(),blank),std::invalid_argument);}
TEST(Stereo,TracksRectifiedImages){StereoTracker tracker;cv::Mat left(240,320,CV_8UC1,cv::Scalar(0)),right;cv::RNG rng(42);for(int i=0;i<100;++i){cv::Point p(rng.uniform(30,300),rng.uniform(20,220));cv::rectangle(left,cv::Rect(p.x,p.y,4,4),cv::Scalar(rng.uniform(100,255)),-1);}cv::Mat warp=(cv::Mat_<double>(2,3)<<1,0,-8,0,1,0);cv::warpAffine(left,right,warp,left.size());auto frame=tracker.track(0,left,right);EXPECT_GT(frame.pixels.size(),10);for(auto& p:frame.pixels)EXPECT_NEAR(p.uv.x()-p.uv.z(),8,.5);}
TEST(Degeneracy,MapProtectionRecoversWithValidButWeakGeometry){
  Degeneracy detector;detector.minimum_points=5;
  EXPECT_TRUE(detector.analyze(Eigen::MatrixXd::Zero(0,6),.01,100,1,true).severe);
  Eigen::MatrixXd planar=Eigen::MatrixXd::Zero(100,6);planar.col(2).setOnes();
  for(int i=0;i<4;++i)EXPECT_TRUE(detector.analyze(planar,.01,100,.01,true).severe);
  auto recovered=detector.analyze(planar,.01,100,.01,true);
  EXPECT_FALSE(recovered.severe);EXPECT_TRUE(recovered.degraded);
  EXPECT_EQ(recovered.reason,DegeneracyStatus::Geometry);
}

TEST(ScanTime,OverlappingScanKeepsNewPointsAndAbsoluteTimes){
  struct Point{double curvature;};struct Cloud{std::vector<Point> points;};
  Cloud cloud{{{0},{1},{3},{50},{103}}};double start=1.;
  ASSERT_TRUE(clipScanToBoundary(cloud,start,1.103,1.002));
  EXPECT_DOUBLE_EQ(start,1.002);ASSERT_EQ(cloud.points.size(),3);
  EXPECT_NEAR(start+cloud.points[0].curvature*.001,1.003,1e-12);
  EXPECT_NEAR(start+cloud.points.back().curvature*.001,1.103,1e-12);
}
TEST(ScanTime,FullyStaleScanIsRejected){
  struct Point{double curvature;};struct Cloud{std::vector<Point> points;};
  Cloud cloud{{{0},{100}}};double start=1.;
  EXPECT_FALSE(clipScanToBoundary(cloud,start,1.1,1.2));
}

TEST(Geodesy,AlignmentRespectsTiltedWorldGravity){
  Mat3 level=expSO3(Vec3(.2,-.1,0));Mat3 expected=level*expSO3(Vec3(0,0,.8));
  for(bool known:{false,true}){
    GnssAlignment alignment;alignment.yaw_known=known;alignment.yaw=.8;
    State state;state.gravity=-9.81*level.col(2);Fix fix;
    alignment.align(fix,state,Vec3::Zero());fix.position=Vec3(10,0,0);state.pose.translation()=expected*fix.position;
    auto result=alignment.align(fix,state,Vec3::Zero());ASSERT_TRUE(result);
    EXPECT_LT((result->position-state.pose.translation()).norm(),1e-8);
  }
}
