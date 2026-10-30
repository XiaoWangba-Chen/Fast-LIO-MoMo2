#include "fast_lio/backend.hpp"
#include "fast_lio/registration.hpp"
#include "fast_lio/map_bundle.hpp"
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LinearContainerFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/linear/NoiseModel.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/io/pcd_io.h>
#include <Eigen/SVD>
#include <thread>
#include <condition_variable>
#include <mutex>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <set>
#include <atomic>
namespace fastlio {
namespace {
gtsam::Pose3 gp(const Pose& T){return gtsam::Pose3(T.matrix());}
Pose ep(const gtsam::Pose3& T){Pose p;p.matrix()=T.matrix();return p;}
gtsam::SharedNoiseModel robust(const Eigen::MatrixXd& P){return gtsam::noiseModel::Robust::Create(gtsam::noiseModel::mEstimator::Huber::Create(1.345),gtsam::noiseModel::Gaussian::Covariance(P));}
// Position residual uses the antenna location, not the IMU origin.
class AntennaFactor:public gtsam::NoiseModelFactor1<gtsam::Pose3>{
  Vec3 measured_,lever_;
 public:
  AntennaFactor(gtsam::Key k,const Fix& f,const Vec3& lever):NoiseModelFactor1(robust(f.covariance),k),measured_(f.position),lever_(lever){}
  gtsam::Vector evaluateError(const gtsam::Pose3& p,boost::optional<gtsam::Matrix&> H=boost::none)const override {return p.transformFrom(lever_,H)-measured_;}
};
Eigen::MatrixXd wheelCovariance(const std::vector<Wheel>& samples,double start,double end,bool nhc){
  double variance=0;
  for(size_t i=1;i<samples.size();++i){double dt=std::max(0.,std::min(end,samples[i].time)-std::max(start,samples[i-1].time));variance+=.5*(samples[i].variance+samples[i-1].variance)*dt*dt;}
  Eigen::MatrixXd P=Eigen::MatrixXd::Identity(nhc?3:1,nhc?3:1)*.25;
  P(0,0)=std::max(1e-4,variance);return P;
}
class WheelFactor:public gtsam::NoiseModelFactor2<gtsam::Pose3,gtsam::Pose3>{
  std::vector<Wheel> samples_;double start_,end_;Pose extrinsic_;bool nhc_;
  gtsam::Vector error(const gtsam::Pose3& a,const gtsam::Pose3& b)const {
    Pose A=ep(a)*extrinsic_,B=ep(b)*extrinsic_;
    Pose relative=A.inverse()*B;
    Eigen::Quaterniond end_rotation(relative.linear());Vec3 displacement=Vec3::Zero();
    for(size_t i=1;i<samples_.size();++i){
      double begin=std::max(start_,samples_[i-1].time),end=std::min(end_,samples_[i].time);if(end<=begin)continue;
      double total=samples_[i].time-samples_[i-1].time;
      double f=(.5*(begin+end)-samples_[i-1].time)/total;
      double speed=(1-f)*samples_[i-1].speed+f*samples_[i].speed;
      double fraction=(.5*(begin+end)-start_)/(end_-start_);
      displacement+=Eigen::Quaterniond::Identity().slerp(fraction,end_rotation)*Vec3(speed*(end-begin),0,0);
    }
    Vec3 r=relative.translation()-displacement;
    if(nhc_)return r;
    return gtsam::Vector1(r.x());
  }
 public:
  WheelFactor(gtsam::Key a,gtsam::Key b,std::vector<Wheel> samples,double start,double end,const BackendConfig& c):
    NoiseModelFactor2(robust(wheelCovariance(samples,start,end,c.nonholonomic)),a,b),samples_(std::move(samples)),start_(start),end_(end),extrinsic_(c.imu_T_base),nhc_(c.nonholonomic){}
  gtsam::Vector evaluateError(const gtsam::Pose3& a,const gtsam::Pose3& b,boost::optional<gtsam::Matrix&> H1=boost::none,boost::optional<gtsam::Matrix&> H2=boost::none)const override{
    auto fn=[this](const gtsam::Pose3& x,const gtsam::Pose3& y){return error(x,y);};
    if(H1)*H1=gtsam::numericalDerivative21<gtsam::Vector,gtsam::Pose3,gtsam::Pose3>(fn,a,b);
    if(H2)*H2=gtsam::numericalDerivative22<gtsam::Vector,gtsam::Pose3,gtsam::Pose3>(fn,a,b);
    return error(a,b);
  }
};
// A completed stereo track is a pose-only factor: eliminate its 3D point at every
// linearization. No landmark variables or independent visual odometry are added.
class StereoFactor:public gtsam::NoiseModelFactor {
  Camera c_;std::vector<Vec3> pixels_;
 public:
  StereoFactor(const gtsam::KeyVector& keys,std::vector<Vec3> pixels,const Camera& c):NoiseModelFactor(gtsam::noiseModel::Isotropic::Sigma(3*keys.size()-3,c.pixel_sigma),keys),c_(c),pixels_(std::move(pixels)){}
  gtsam::Vector unwhitenedError(const gtsam::Values& values,boost::optional<std::vector<gtsam::Matrix>&> matrices=boost::none)const override {
    const size_t n=keys().size();std::vector<Pose> poses;for(auto key:keys())poses.push_back(ep(values.at<gtsam::Pose3>(key)));
    Vec3 world=poses[0]*c_.body_T_camera*c_.triangulate(pixels_[0]);
    auto projectJac=[this](const Vec3& p){Mat3 J;J<<c_.fx/p.z(),0,-c_.fx*p.x()/(p.z()*p.z()),0,c_.fy/p.z(),-c_.fy*p.y()/(p.z()*p.z()),c_.fx/p.z(),0,-c_.fx*(p.x()-c_.baseline)/(p.z()*p.z());return J;};
    bool valid=true;
    for(int iteration=0;iteration<4;++iteration){Mat3 H=Mat3::Zero();Vec3 b=Vec3::Zero();for(size_t i=0;i<n;++i){Pose T=poses[i]*c_.body_T_camera;Vec3 p=T.inverse()*world;if(p.z()<.2){valid=false;break;}Mat3 J=projectJac(p)*T.linear().transpose();H+=J.transpose()*J;b+=J.transpose()*(pixels_[i]-c_.project(p));}if(!valid)break;Vec3 dx=H.ldlt().solve(b);if(!dx.allFinite()){valid=false;break;}world+=dx;if(dx.norm()<1e-5)break;}
    const int m=3*n;Eigen::MatrixXd Hf=Eigen::MatrixXd::Zero(m,3),Hx=Eigen::MatrixXd::Zero(m,6*n);Eigen::VectorXd r=Eigen::VectorXd::Zero(m);
    for(size_t i=0;i<n&&valid;++i){const Pose T=poses[i]*c_.body_T_camera;Vec3 p=T.inverse()*world;if(p.z()<.2||p.z()>150){valid=false;break;}Vec3 residual=c_.project(p)-pixels_[i];if(residual.norm()>5*c_.pixel_sigma){valid=false;break;}Mat3 J=projectJac(p);Hf.block<3,3>(3*i,0)=J*T.linear().transpose();
      // Pose3 local coordinates are body rotation followed by body translation.
      Hx.block<3,3>(3*i,6*i)=J*c_.body_T_camera.linear().transpose()*skew(poses[i].inverse()*world);
      Hx.block<3,3>(3*i,6*i+3)=-J*c_.body_T_camera.linear().transpose();r.segment<3>(3*i)=residual;}
    if(!valid){if(matrices)matrices->assign(n,Eigen::MatrixXd::Zero(m-3,6));return Eigen::VectorXd::Zero(m-3);}
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(Hf,Eigen::ComputeFullU);Eigen::MatrixXd A=svd.matrixU().rightCols(m-3);
    if(matrices){matrices->resize(n);for(size_t i=0;i<n;++i)(*matrices)[i]=A.transpose()*Hx.middleCols(6*i,6);}
    return A.transpose()*r;
  }
};
}
struct Backend::Impl {
  BackendConfig config;std::function<void(const BackendResult&)> callback;
  mutable std::mutex mutex;std::condition_variable changed,idle;std::deque<BackendEvent> queue;
  bool stopping=false,busy=false;std::thread worker;
  BackendResult result;uint64_t desired_session=0;std::atomic<uint64_t> dropped{0};
  gtsam::ISAM2 isam;gtsam::Values estimate;std::vector<State> nodes;
  std::vector<size_t> odometry_factors;
  uint64_t wheel_count=0,gnss_count=0,visual_count=0;
  std::mutex graph_mutex;
  struct Keyframe{size_t node;Cloud::ConstPtr cloud;Descriptor descriptor;};std::vector<Keyframe> keyframes;
  std::vector<Wheel> wheels;
  std::map<uint64_t,std::vector<std::pair<size_t,Vec3>>> tracks;
  explicit Impl(const BackendConfig& c,std::function<void(const BackendResult&)> cb):config(c),callback(std::move(cb)){
    worker=std::thread([this]{run();});
  }
  ~Impl(){{std::lock_guard<std::mutex> l(mutex);stopping=true;changed.notify_all();}worker.join();}
  void clear(uint64_t session){isam=gtsam::ISAM2();estimate.clear();nodes.clear();odometry_factors.clear();wheel_count=gnss_count=visual_count=0;keyframes.clear();wheels.clear();tracks.clear();result=BackendResult();result.session=session;}
  void publish(const BackendEvent& event,bool build_map){
    BackendResult next;{std::lock_guard<std::mutex>l(mutex);next=result;}
    next.local=event.state;next.map_T_odom=ep(estimate.at<gtsam::Pose3>(nodes.size()-1))*event.state.pose.inverse();
    next.trajectory.clear();for(size_t i=0;i<nodes.size();++i)next.trajectory.emplace_back(nodes[i].time,ep(estimate.at<gtsam::Pose3>(i)));
    auto P=isam.marginalCovariance(nodes.size()-1); // Pose3 local [rotation,translation] -> map [position,rotation]
    Mat6 J=Mat6::Zero();Mat3 R=estimate.at<gtsam::Pose3>(nodes.size()-1).rotation().matrix();J.topRightCorner<3,3>()=R;J.bottomLeftCorner<3,3>()=Mat3::Identity();next.covariance=J*P*J.transpose();
    if(build_map){next.map.reset(new Cloud);for(const auto& k:keyframes){Cloud transformed;pcl::transformPointCloud(*k.cloud,transformed,estimate.at<gtsam::Pose3>(k.node).matrix().cast<float>());*next.map+=transformed;}pcl::VoxelGrid<pcl::PointXYZI> voxel;voxel.setLeafSize(config.voxel,config.voxel,config.voxel);voxel.setInputCloud(next.map);Cloud filtered;voxel.filter(filtered);*next.map=filtered;}
    ++next.processed;next.dropped=dropped.load();next.wheel_factors=wheel_count;next.gnss_factors=gnss_count;next.visual_factors=visual_count;
    {std::lock_guard<std::mutex>l(mutex);if(event.session!=desired_session)return;result=next;}
    if(callback)callback(next);
  }
  void process(const BackendEvent& e){
    if(e.session!=result.session)clear(e.session);
    if(e.camera)config.camera=*e.camera;
    if(e.wheel){wheels.push_back(*e.wheel);while(wheels.size()>10000)wheels.erase(wheels.begin());return;}
    bool make_keyframe=false;
    if(e.cloud){
      if(keyframes.empty())make_keyframe=true;
      else {Pose d=nodes[keyframes.back().node].pose.inverse()*e.state.pose;make_keyframe=d.translation().norm()>=config.key_distance||Eigen::AngleAxisd(d.linear()).angle()>=config.key_angle||e.state.time-nodes[keyframes.back().node].time>=config.key_seconds;}
    }
    if(!make_keyframe&&!e.stereo&&!e.fix)return;
    if(!nodes.empty()&&e.state.time<nodes.back().time-1e-8)return;
    gtsam::NonlinearFactorGraph graph;gtsam::Values initial;gtsam::FactorIndices removed;std::optional<size_t> new_odometry;
    size_t index=nodes.size();bool same=!nodes.empty()&&std::abs(e.state.time-nodes.back().time)<1e-8;
    if(same)--index;
    if(same&&make_keyframe&&index>0){
      const Pose relative=nodes[index-1].pose.inverse()*e.state.pose;
      Mat6 P=4*(e.state.covariance.topLeftCorner<6,6>()+nodes[index-1].covariance.topLeftCorner<6,6>()+Mat6::Identity()*.001);
      Mat6 J=Mat6::Zero();J.topRightCorner<3,3>()=Mat3::Identity();J.bottomLeftCorner<3,3>()=nodes[index-1].pose.linear().transpose();
      removed.push_back(odometry_factors[index]);new_odometry=graph.size();
      graph.add(gtsam::BetweenFactor<gtsam::Pose3>(index-1,index,gp(relative),robust(J*P*J.transpose())));nodes[index]=e.state;
    }
    if(!same) {
      if(nodes.empty())graph.add(gtsam::PriorFactor<gtsam::Pose3>(0,gp(e.state.pose),gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector6()<<.1,.1,.1,.01,.01,.01).finished())));
      else {
        Pose relative=nodes.back().pose.inverse()*e.state.pose;
        Mat6 local_cov=e.state.covariance.topLeftCorner<6,6>()+nodes.back().covariance.topLeftCorner<6,6>();
        local_cov+=Mat6::Identity()*.001;
        // LIO marginal states are correlated. Inflate the approximation rather than
        // pretending two marginals define an independent relative covariance.
        local_cov*=4.;
        if(e.degeneracy.degraded)local_cov+=Mat6::Identity()*.1;
        Mat6 J=Mat6::Zero();J.topRightCorner<3,3>()=Mat3::Identity();J.bottomLeftCorner<3,3>()=nodes.back().pose.linear().transpose();
        new_odometry=graph.size();graph.add(gtsam::BetweenFactor<gtsam::Pose3>(index-1,index,gp(relative),robust(J*local_cov*J.transpose())));
        if(config.side==Side::Backend&&wheels.size()>1){
          double start=nodes.back().time,end=e.state.time;bool covered=wheels.front().time<=start&&wheels.back().time>=end-.02;
          for(size_t i=1;i<wheels.size();++i)if(wheels[i].time>start&&wheels[i-1].time<end&&wheels[i].time-wheels[i-1].time>.1)covered=false;
          if(covered){auto samples=wheels;if(samples.back().time<end){auto last=samples.back();last.time=end;samples.push_back(last);}double distance=0;for(size_t i=1;i<samples.size();++i){double dt=std::max(0.,std::min(end,samples[i].time)-std::max(start,samples[i-1].time));distance+=.5*(samples[i].speed+samples[i-1].speed)*dt;}
            if(std::abs(relative.translation().norm()-std::abs(distance))<std::max(1.,.5*std::abs(distance))){graph.add(boost::make_shared<WheelFactor>(index-1,index,samples,start,end,config));++wheel_count;}}
          while(wheels.size()>2&&wheels[1].time<=end)wheels.erase(wheels.begin());
        }
      }
      initial.insert(index,gp(e.state.pose));nodes.push_back(e.state);odometry_factors.push_back(0);
    }
    if(config.side==Side::Backend&&e.fix){Vec3 predicted=e.state.pose*config.antenna;Eigen::Matrix3d S=e.fix->covariance+e.state.covariance.topLeftCorner<3,3>();Vec3 r=e.fix->position-predicted;if(r.dot(S.ldlt().solve(r))<27){graph.add(boost::make_shared<AntennaFactor>(index,*e.fix,config.antenna));++gnss_count;}}
    if(config.side==Side::Backend&&e.stereo&&config.camera.valid()){
      std::set<uint64_t> present;
      for(const auto& pixel:e.stereo->pixels){present.insert(pixel.feature);tracks[pixel.feature].emplace_back(index,pixel.uv);}
      for(auto it=tracks.begin();it!=tracks.end();){if(!present.count(it->first)||it->second.size()>=static_cast<size_t>(config.visual_track_length)){
        if(it->second.size()>=2){gtsam::KeyVector keys;std::vector<Vec3> uv;for(const auto& obs:it->second){keys.push_back(obs.first);uv.push_back(obs.second);}graph.add(boost::make_shared<StereoFactor>(keys,uv,config.camera));++visual_count;}it=tracks.erase(it);
      }else ++it;}
    }
    bool loop=false;
    if(make_keyframe){
      Keyframe current{index,e.cloud,descriptor(*e.cloud)};
      if(config.loop_enabled&&keyframes.size()>static_cast<size_t>(config.loop_separation)&&keyframes.size()%config.loop_interval==0){
        double best=config.loop_distance;int candidate=-1,shift=0;
        Pose predicted_current=e.state.pose;
        if(index>0)predicted_current=ep(estimate.at<gtsam::Pose3>(index-1))*nodes[index-1].pose.inverse()*e.state.pose;
        for(size_t i=0;i+config.loop_separation<keyframes.size();++i){
          const Pose historical=ep(estimate.at<gtsam::Pose3>(keyframes[i].node));
          // A candidate farther than this cannot pass the translation and
          // correction gates below (triangle inequality). Exclude it before
          // descriptor ranking so an impossible alias cannot hide a valid loop.
          if((historical.translation()-predicted_current.translation()).norm()>=config.loop_max_translation+config.loop_max_correction)continue;
          auto d=descriptorDistance(current.descriptor,keyframes[i].descriptor);
          if(d.first<best){best=d.first;candidate=i;shift=d.second;}
        }
        if(candidate>=0){
          const auto& target=keyframes[candidate];Cloud::Ptr submap(new Cloud);
          Pose target_pose=ep(estimate.at<gtsam::Pose3>(target.node));
          for(int i=std::max(0,candidate-2);i<=std::min(int(keyframes.size())-1,candidate+2);++i){Cloud local;pcl::transformPointCloud(*keyframes[i].cloud,local,(target_pose.inverse()*ep(estimate.at<gtsam::Pose3>(keyframes[i].node))).matrix().cast<float>());*submap+=local;}
          pcl::IterativeClosestPoint<pcl::PointXYZI,pcl::PointXYZI> icp;icp.setInputSource(current.cloud);icp.setInputTarget(submap);icp.setMaximumIterations(60);icp.setMaxCorrespondenceDistance(2.);icp.setTransformationEpsilon(1e-6);
          Pose current_pose=e.state.pose;
          if(index>0)current_pose=ep(estimate.at<gtsam::Pose3>(index-1))*nodes[index-1].pose.inverse()*e.state.pose;
          Pose predicted=target_pose.inverse()*current_pose,guess=predicted;
          double predicted_yaw=std::atan2(guess.linear()(1,0),guess.linear()(0,0));
          // current sector j matches historical sector j+shift: rotate the
          // current cloud by +shift to express it in the historical frame.
          guess.linear()=Eigen::AngleAxisd(shift*2*M_PI/60-predicted_yaw,Vec3::UnitZ()).toRotationMatrix()*guess.linear();
          Cloud aligned;icp.align(aligned,guess.matrix().cast<float>());
          pcl::KdTreeFLANN<pcl::PointXYZI> tree;tree.setInputCloud(submap);int matches=0;std::vector<int> ids(1);std::vector<float> distances(1);for(const auto& p:aligned)if(tree.nearestKSearch(p,1,ids,distances)&&distances[0]<.25)++matches;
          double overlap=double(matches)/std::max<size_t>(1,aligned.size());Pose matched;matched.matrix()=icp.getFinalTransformation().cast<double>();
          if(icp.hasConverged()&&icp.getFitnessScore()<config.icp_fitness&&overlap>=config.loop_overlap&&matched.translation().norm()<config.loop_max_translation&&(predicted.inverse()*matched).translation().norm()<config.loop_max_correction&&Eigen::AngleAxisd(predicted.linear().transpose()*matched.linear()).angle()<config.loop_max_rotation){graph.add(gtsam::BetweenFactor<gtsam::Pose3>(target.node,index,gp(matched),robust(Mat6::Identity()*.01)));loop=true;}
        }
      }
      keyframes.push_back(std::move(current));
    }
    auto update=isam.update(graph,initial,removed);if(new_odometry)odometry_factors[index]=update.newFactorsIndices[*new_odometry];if(loop)isam.update();estimate=isam.calculateEstimate();
    if(loop){std::lock_guard<std::mutex>l(mutex);++result.loops;}
    publish(e,loop||(make_keyframe&&keyframes.size()%10==1));
  }
  void run(){for(;;){BackendEvent event;{std::unique_lock<std::mutex>l(mutex);changed.wait(l,[this]{return stopping||!queue.empty();});if(stopping&&queue.empty())break;event=std::move(queue.front());queue.pop_front();busy=true;if(event.session!=desired_session){busy=false;idle.notify_all();continue;}}
    try{std::lock_guard<std::mutex> graph_lock(graph_mutex);process(event);}catch(const std::exception& error){std::lock_guard<std::mutex>l(mutex);result.error=error.what();queue.clear();/* Stop accepting this session: a failed iSAM update is not transactional. */failed_session=event.session;}
    {std::lock_guard<std::mutex>l(mutex);busy=false;idle.notify_all();}}
  }
  uint64_t failed_session=UINT64_MAX;
};
Backend::Backend(const BackendConfig& c,std::function<void(const BackendResult&)> cb):impl_(new Impl(c,std::move(cb))){}
Backend::~Backend()=default;
bool Backend::submit(BackendEvent e){std::lock_guard<std::mutex>l(impl_->mutex);if(e.session!=impl_->desired_session||e.session==impl_->failed_session)return false;if(impl_->queue.size()>=impl_->config.max_queue){++impl_->dropped;return false;}impl_->queue.push_back(std::move(e));impl_->changed.notify_one();return true;}
void Backend::reset(uint64_t session){flush();std::lock_guard<std::mutex> graph_lock(impl_->graph_mutex);std::lock_guard<std::mutex>l(impl_->mutex);impl_->desired_session=session;impl_->failed_session=UINT64_MAX;impl_->clear(session);}
BackendResult Backend::snapshot()const{std::lock_guard<std::mutex>l(impl_->mutex);return impl_->result;}
void Backend::flush(){std::unique_lock<std::mutex>l(impl_->mutex);impl_->idle.wait(l,[this]{return impl_->queue.empty()&&!impl_->busy;});}
bool Backend::save(std::string& error){flush();std::lock_guard<std::mutex> graph_lock(impl_->graph_mutex);auto result=snapshot();try{std::filesystem::create_directories(impl_->config.output_directory);std::ofstream out(impl_->config.output_directory+"/trajectory.tum");if(!out)throw std::runtime_error("Cannot open trajectory output");for(auto& pair:result.trajectory){Pose base=pair.second*impl_->config.imu_T_base;Eigen::Quaterniond q(base.linear());const Vec3 p=base.translation();out<<std::fixed<<std::setprecision(9)<<pair.first<<" "<<p.transpose()<<" "<<q.x()<<" "<<q.y()<<" "<<q.z()<<" "<<q.w()<<"\n";}
  Cloud map;for(const auto& k:impl_->keyframes){Cloud transformed;pcl::transformPointCloud(*k.cloud,transformed,impl_->estimate.at<gtsam::Pose3>(k.node).matrix().cast<float>());map+=transformed;}
  if(!map.empty()&&pcl::io::savePCDFileBinary(impl_->config.output_directory+"/map.pcd",map)<0)throw std::runtime_error("PCD write failed");
  std::ofstream keyframes(impl_->config.output_directory+"/keyframes.csv");keyframes<<"node,timestamp,points\n";for(const auto& k:impl_->keyframes){keyframes<<k.node<<","<<std::setprecision(17)<<impl_->nodes[k.node].time<<","<<k.cloud->size()<<"\n";if(pcl::io::savePCDFileBinary(impl_->config.output_directory+"/keyframe_"+std::to_string(k.node)+".pcd",*k.cloud)<0)throw std::runtime_error("Keyframe write failed");}
  std::vector<MapKeyframe> frames;for(const auto& k:impl_->keyframes)frames.push_back({k.node,impl_->nodes[k.node].time,ep(impl_->estimate.at<gtsam::Pose3>(k.node)),k.cloud});
  saveMapBundle(impl_->config.output_directory+"/map_bundle",frames);return true;
}catch(const std::exception& e){error=e.what();return false;}}
}
