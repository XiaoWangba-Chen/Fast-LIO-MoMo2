#include "fast_lio/runtime.hpp"
#include "fast_lio/stereo.hpp"
#include "fast_lio/ros2_support.hpp"
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <pcl_conversions/pcl_conversions.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <variant>
#include <thread>
#include <atomic>
#include <set>
#include <Eigen/Eigenvalues>
namespace fastlio {
namespace {
geometry_msgs::msg::Pose poseMessage(const Pose& p){geometry_msgs::msg::Pose out;Eigen::Quaterniond q(p.linear());out.position.x=p.translation().x();out.position.y=p.translation().y();out.position.z=p.translation().z();out.orientation.x=q.x();out.orientation.y=q.y();out.orientation.z=q.z();out.orientation.w=q.w();return out;}
geometry_msgs::msg::TransformStamped transformMessage(const Pose& p,double time,const std::string& parent,const std::string& child){geometry_msgs::msg::TransformStamped out;out.header.stamp=stamp_from_seconds(time);out.header.frame_id=parent;out.child_frame_id=child;auto pose=poseMessage(p);out.transform.translation.x=pose.position.x;out.transform.translation.y=pose.position.y;out.transform.translation.z=pose.position.z;out.transform.rotation=pose.orientation;return out;}
Pose extrinsic(const rclcpp::Node::SharedPtr& node,const std::string& name){
  auto t=node->declare_parameter<std::vector<double>>(name+".translation",{0.,0.,0.});
  auto q=node->declare_parameter<std::vector<double>>(name+".quaternion",{0.,0.,0.,1.});
  if(t.size()!=3||q.size()!=4)throw std::invalid_argument(name+": require 3 translation and 4 quaternion entries [x,y,z,w]");
  Eigen::Quaterniond rotation(q[3],q[0],q[1],q[2]);if(!rotation.coeffs().allFinite()||std::abs(rotation.norm()-1)>1e-3)throw std::invalid_argument(name+": quaternion must be finite and unit length");
  Pose p=Pose::Identity();p.translation()=Vec3(t[0],t[1],t[2]);p.linear()=rotation.normalized().toRotationMatrix();if(!p.matrix().allFinite())throw std::invalid_argument("Non-finite extrinsic");return p;
}
Mat6 basePoseCovariance(const Pose& imu_pose,const Pose& imu_T_base,const Mat6& covariance){
  Mat6 J=Mat6::Zero();J.topLeftCorner<3,3>()=Mat3::Identity();
  J.topRightCorner<3,3>()=-imu_pose.linear()*skew(imu_T_base.translation());
  J.bottomRightCorner<3,3>()=imu_pose.linear();
  return J*covariance*J.transpose();
}
nav_msgs::msg::Odometry odometry(const State& s,const Pose& imu_T_base,const std::string& frame,const Vec3& omega,double gyro_variance){
  nav_msgs::msg::Odometry o;o.header.stamp=stamp_from_seconds(s.time);o.header.frame_id=frame;o.child_frame_id="base_link";
  Pose base=s.pose*imu_T_base;o.pose.pose=poseMessage(base);
  Mat6 P=basePoseCovariance(s.pose,imu_T_base,s.covariance.topLeftCorner<6,6>());
  Vec3 v=imu_T_base.linear().transpose()*(s.pose.linear().transpose()*s.velocity+(omega-s.bg).cross(imu_T_base.translation()));
  Vec3 w=imu_T_base.linear().transpose()*(omega-s.bg);
  o.twist.twist.linear.x=v.x();o.twist.twist.linear.y=v.y();o.twist.twist.linear.z=v.z();o.twist.twist.angular.x=w.x();o.twist.twist.angular.y=w.y();o.twist.twist.angular.z=w.z();
  Eigen::Matrix<double,6,23> V=Eigen::Matrix<double,6,23>::Zero();const Mat3 B=imu_T_base.linear().transpose();
  V.block<3,3>(0,3)=B*skew(s.pose.linear().transpose()*s.velocity);V.block<3,3>(0,12)=B*s.pose.linear().transpose();V.block<3,3>(0,15)=B*skew(imu_T_base.translation());V.block<3,3>(3,15)=-B;
  Mat6 velocity_cov=V*s.covariance*V.transpose();
  Eigen::Matrix<double,6,3> gyro_jacobian;
  gyro_jacobian.topRows<3>()=-B*skew(imu_T_base.translation());
  gyro_jacobian.bottomRows<3>()=B;
  velocity_cov+=gyro_variance*gyro_jacobian*gyro_jacobian.transpose();
  for(int i=0;i<6;++i)for(int j=0;j<6;++j){o.pose.covariance[6*i+j]=P(i,j);o.twist.covariance[6*i+j]=velocity_cov(i,j);}
  return o;
}
struct ImagePair{double time; sensor_msgs::msg::Image::ConstSharedPtr left,right;};
using Auxiliary=std::variant<Wheel,Fix,ImagePair>;
double eventTime(const Auxiliary& event){return std::visit([](const auto& e){return e.time;},event);}
}
struct Runtime::Impl {
  Runtime& owner;rclcpp::Node::SharedPtr parameters,io;Side side;
  bool wheel_enabled,gnss_enabled,stereo_enabled,backend_enabled;
  Pose imu_T_base,camera_raw;Vec3 antenna;Camera camera;
  cv::Mat left_mapx,left_mapy,right_mapx,right_mapy;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr immutable_parameters;
  rclcpp::TimerBase::SharedPtr watchdog;
  std::atomic<int64_t> last_imu_wall{0};uint64_t invalid_imu=0;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_service;WheelModel wheel_model;
  Geodesy geodesy;GnssAlignment alignment;StereoTracker tracker;
  std::mutex mutex;std::deque<Auxiliary> events;std::set<std::pair<size_t,int64_t>> observed_ids;std::deque<sensor_msgs::msg::Image::ConstSharedPtr> left_images,right_images;
  sensor_msgs::msg::CameraInfo::ConstSharedPtr left_info,right_info;
  Audit wheel_audit,gnss_audit,stereo_audit;uint64_t stereo_updates=0;size_t stereo_features=0;
  double gyro_measurement_variance=0;
  double last_processed=-1,last_imu=-1,imu_offset=0,wheel_offset=0,gnss_offset=0,left_offset=0,right_offset=0,sync_tolerance=.005;
  std::atomic<double> fast_time{-1};
  size_t max_events=2000;std::atomic<bool> reset_needed{false};uint64_t session=0;
  std::unique_ptr<Backend> backend;BackendConfig backend_config;
  rclcpp::executors::SingleThreadedExecutor executor;std::thread io_thread;
  std::vector<rclcpp::SubscriptionBase::SharedPtr> subscriptions;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr fast_pub,local_pub,global_pub;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_tf;
  explicit Impl(Runtime& owner,rclcpp::Node::SharedPtr node):owner(owner),parameters(node),io(std::make_shared<rclcpp::Node>("fastlio_io",rclcpp::NodeOptions().arguments({"--ros-args","-r","__node:=fastlio_io"}))),side(parseSide(node->declare_parameter<std::string>("fusion.mode","backend"))){
    wheel_enabled=node->declare_parameter<bool>("wheel.enabled",false);gnss_enabled=node->declare_parameter<bool>("gnss.enabled",false);stereo_enabled=node->declare_parameter<bool>("stereo.enabled",false);
    backend_enabled=node->declare_parameter<bool>("backend.enabled",true);
    if(side==Side::Backend&&!backend_enabled&&(wheel_enabled||gnss_enabled||stereo_enabled))throw std::invalid_argument("fusion.mode=backend requires backend.enabled=true when auxiliary sensors are enabled");
    io->set_parameter(rclcpp::Parameter("use_sim_time",node->get_parameter("use_sim_time").as_bool()));
    imu_T_base=extrinsic(node,"extrinsics.imu_T_base");camera.body_T_camera=extrinsic(node,"extrinsics.imu_T_camera");camera_raw=camera.body_T_camera;auto antenna_pose=extrinsic(node,"extrinsics.imu_T_gnss");antenna=antenna_pose.translation();
    alignment.yaw_known=node->declare_parameter<bool>("gnss.yaw_known",false);alignment.yaw=node->declare_parameter<double>("gnss.enu_to_odom_yaw",0.);alignment.minimum_baseline=node->declare_parameter<double>("gnss.alignment_baseline",5.);
    wheel_model.imu_T_base=imu_T_base;wheel_model.nonholonomic=node->declare_parameter<bool>("wheel.nonholonomic",true);
    wheel_model.lateral_variance=node->declare_parameter<double>("wheel.lateral_variance",.25);wheel_model.vertical_variance=node->declare_parameter<double>("wheel.vertical_variance",.25);
    gyro_measurement_variance=node->get_parameter("mapping.gyr_cov").as_double();
    imu_offset=node->get_parameter("common.time_offset_lidar_to_imu").as_double();
    wheel_offset=node->declare_parameter<double>("wheel.time_offset",0.);gnss_offset=node->declare_parameter<double>("gnss.time_offset",0.);left_offset=node->declare_parameter<double>("stereo.left_time_offset",0.);right_offset=node->declare_parameter<double>("stereo.right_time_offset",0.);
    sync_tolerance=node->declare_parameter<double>("stereo.sync_tolerance",.005);camera.pixel_sigma=node->declare_parameter<double>("stereo.pixel_sigma",1.);
    int clone_count=node->declare_parameter<int>("stereo.max_clones",10);
    if(clone_count<2||clone_count>30)throw std::invalid_argument("stereo.max_clones must be between 2 and 30");
    owner.filter.max_clones=clone_count;
    owner.propagator.max_gap=node->declare_parameter<double>("imu.max_gap",.1);
    owner.degeneracy.enabled=node->declare_parameter<bool>("degeneracy.enabled",true);
    owner.degeneracy.rotation_scale=node->declare_parameter<double>("degeneracy.rotation_scale",10.);
    owner.degeneracy.relative_threshold=node->declare_parameter<double>("degeneracy.relative_threshold",.01);
    owner.degeneracy.absolute_threshold=node->declare_parameter<double>("degeneracy.absolute_threshold",.01);
    owner.degeneracy.minimum_match_ratio=node->declare_parameter<double>("degeneracy.minimum_match_ratio",.05);
    owner.degeneracy.maximum_residual=node->declare_parameter<double>("degeneracy.maximum_residual",.3);
    owner.degeneracy.enter_frames=node->declare_parameter<int>("degeneracy.enter_frames",2);owner.degeneracy.exit_frames=node->declare_parameter<int>("degeneracy.exit_frames",5);
    if(owner.filter.max_clones<2||owner.propagator.max_gap<=0||owner.degeneracy.rotation_scale<=0||sync_tolerance<=0||camera.pixel_sigma<=0||alignment.minimum_baseline<=0)throw std::invalid_argument("Invalid fusion numerical parameters");
    fast_pub=io->create_publisher<nav_msgs::msg::Odometry>("/odometry/imu",100);local_pub=io->create_publisher<nav_msgs::msg::Odometry>("/odometry/local",10);global_pub=io->create_publisher<nav_msgs::msg::Odometry>("/odometry/global",10);
    path_pub=io->create_publisher<nav_msgs::msg::Path>("/path/optimized",1);map_pub=io->create_publisher<sensor_msgs::msg::PointCloud2>("/map/global",rclcpp::QoS(1).transient_local());diagnostics_pub=io->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics",10);
    tf=std::make_unique<tf2_ros::TransformBroadcaster>(io);static_tf=std::make_unique<tf2_ros::StaticTransformBroadcaster>(io);
    std::vector<geometry_msgs::msg::TransformStamped> transforms;
    transforms.push_back(transformMessage(imu_T_base.inverse(),0,"base_link","imu_link"));
    transforms.push_back(transformMessage(imu_T_base.inverse()*camera.body_T_camera,0,"base_link","camera_left_optical"));
    transforms.push_back(transformMessage(imu_T_base.inverse()*antenna_pose,0,"base_link","gnss_link"));
    auto lidar_t=node->get_parameter("mapping.extrinsic_T").as_double_array();auto lidar_r=node->get_parameter("mapping.extrinsic_R").as_double_array();Pose lidar=Pose::Identity();if(lidar_t.size()==3&&lidar_r.size()==9){lidar.translation()=Vec3(lidar_t[0],lidar_t[1],lidar_t[2]);for(int i=0;i<3;++i)for(int j=0;j<3;++j)lidar.linear()(i,j)=lidar_r[3*i+j];transforms.push_back(transformMessage(imu_T_base.inverse()*lidar,0,"base_link","lidar_link"));}static_tf->sendTransform(transforms);
    subscriptions.push_back(io->create_subscription<sensor_msgs::msg::Imu>(node->get_parameter("common.imu_topic").as_string(),rclcpp::SensorDataQoS().keep_last(2000),[this](sensor_msgs::msg::Imu::ConstSharedPtr m){
      std::lock_guard<std::mutex> imu_lock(mutex);
      Imu sample{stamp_seconds(m->header.stamp)-imu_offset,Vec3(m->linear_acceleration.x,m->linear_acceleration.y,m->linear_acceleration.z),Vec3(m->angular_velocity.x,m->angular_velocity.y,m->angular_velocity.z)};
      if(!sample.acceleration.allFinite()||!sample.angular_velocity.allFinite()||sample.time<0){++invalid_imu;return;}
      if(sample.time<last_imu){reset_needed=true;return;}if(sample.time==last_imu)return;last_imu=sample.time;last_imu_wall=std::chrono::steady_clock::now().time_since_epoch().count();
      if(reset_needed)return;
      auto propagated=this->owner.propagator.push(sample);fast_time=sample.time;
      if(propagated){fast_pub->publish(odometry(*propagated,imu_T_base,"odom",sample.angular_velocity,gyro_measurement_variance));tf->sendTransform(transformMessage(propagated->pose*imu_T_base,propagated->time,"odom","base_link"));}
    }));
    if(wheel_enabled)subscriptions.push_back(io->create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(node->declare_parameter<std::string>("wheel.topic","/wheel/twist"),rclcpp::SensorDataQoS(),[this](geometry_msgs::msg::TwistWithCovarianceStamped::ConstSharedPtr m){std::lock_guard<std::mutex>l(mutex);Wheel w{stamp_seconds(m->header.stamp)+wheel_offset,m->twist.twist.linear.x,m->twist.covariance[0]};if(!std::isfinite(w.speed)||!std::isfinite(w.variance)||w.variance<=0||m->header.frame_id!="base_link"){++wheel_audit.rejected;return;}enqueue(w,wheel_audit);}));
    if(gnss_enabled)subscriptions.push_back(io->create_subscription<sensor_msgs::msg::NavSatFix>(node->declare_parameter<std::string>("gnss.topic","/gps/fix"),rclcpp::SensorDataQoS(),[this](sensor_msgs::msg::NavSatFix::ConstSharedPtr m){std::lock_guard<std::mutex>l(mutex);if(m->status.status<0||m->position_covariance_type==0){++gnss_audit.rejected;return;}Fix f;f.time=stamp_seconds(m->header.stamp)+gnss_offset;for(int i=0;i<3;++i)for(int j=0;j<3;++j)f.covariance(i,j)=m->position_covariance[3*i+j];Eigen::SelfAdjointEigenSolver<Mat3> solver(f.covariance);if(!f.covariance.allFinite()||!f.covariance.isApprox(f.covariance.transpose())||solver.eigenvalues().minCoeff()<=0||solver.eigenvalues().maxCoeff()>2500){++gnss_audit.rejected;return;}try{f.position=geodesy.convert(m->latitude,m->longitude,m->altitude);enqueue(f,gnss_audit);}catch(...){++gnss_audit.rejected;}}));
    if(stereo_enabled){
      auto image_qos=rclcpp::SensorDataQoS().keep_last(10);
      if(node->get_parameter("common.reliable_inputs").as_bool())image_qos.reliable();
      subscriptions.push_back(io->create_subscription<sensor_msgs::msg::CameraInfo>(node->declare_parameter<std::string>("stereo.left_info_topic","/camera/left/camera_info"),rclcpp::QoS(10),[this](sensor_msgs::msg::CameraInfo::ConstSharedPtr m){std::lock_guard<std::mutex>l(mutex);if(!left_info)left_info=m;}));
      subscriptions.push_back(io->create_subscription<sensor_msgs::msg::CameraInfo>(node->declare_parameter<std::string>("stereo.right_info_topic","/camera/right/camera_info"),rclcpp::QoS(10),[this](sensor_msgs::msg::CameraInfo::ConstSharedPtr m){std::lock_guard<std::mutex>l(mutex);if(!right_info)right_info=m;}));
      for(bool left:{true,false})subscriptions.push_back(io->create_subscription<sensor_msgs::msg::Image>(node->declare_parameter<std::string>(left?"stereo.left_topic":"stereo.right_topic",left?"/camera/left/image_raw":"/camera/right/image_raw"),image_qos,[this,left](sensor_msgs::msg::Image::ConstSharedPtr m){std::lock_guard<std::mutex>l(mutex);auto& q=left?left_images:right_images;q.push_back(m);while(q.size()>20){q.pop_front();++stereo_audit.overflow;}while(!left_images.empty()&&!right_images.empty()){
        double a=stamp_seconds(left_images.front()->header.stamp)+left_offset,b=stamp_seconds(right_images.front()->header.stamp)+right_offset;
        if(std::abs(a-b)<=sync_tolerance){enqueue(ImagePair{a,left_images.front(),right_images.front()},stereo_audit);left_images.pop_front();right_images.pop_front();}else if(a<b){left_images.pop_front();++stereo_audit.rejected;}else{right_images.pop_front();++stereo_audit.rejected;}
      }}));
    }
    backend_config.enabled=backend_enabled;backend_config.side=side;backend_config.imu_T_base=imu_T_base;backend_config.antenna=antenna;backend_config.nonholonomic=wheel_model.nonholonomic;
    backend_config.loop_enabled=node->declare_parameter<bool>("backend.loop_enabled",true);backend_config.key_distance=node->declare_parameter<double>("backend.key_distance",1.);backend_config.key_angle=node->declare_parameter<double>("backend.key_angle",.2);backend_config.key_seconds=node->declare_parameter<double>("backend.key_seconds",1.);backend_config.output_directory=node->declare_parameter<std::string>("output.directory","/tmp/fastlio-output");
    immutable_parameters=node->add_on_set_parameters_callback([this](const std::vector<rclcpp::Parameter>& updates){
      rcl_interfaces::msg::SetParametersResult result;result.successful=true;
      for(const auto& p:updates)if(p.get_name()=="fusion.mode"&&(p.get_type()!=rclcpp::ParameterType::PARAMETER_STRING||p.as_string()!=(side==Side::Frontend?"frontend":"backend"))){result.successful=false;result.reason="fusion.mode is immutable; restart the estimator to change it";}
      return result;
    });
    watchdog=io->create_wall_timer(std::chrono::milliseconds(50),[this]{
      auto received=last_imu_wall.load();if(!received)return;
      auto elapsed=std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()-std::chrono::steady_clock::duration(received)).count();
      if(elapsed<=this->owner.propagator.max_gap)return;
      diagnostic_msgs::msg::DiagnosticArray msg;msg.header.stamp=io->now();diagnostic_msgs::msg::DiagnosticStatus status;status.name="fastlio/imu";status.hardware_id="fastlio";status.level=2;status.message="IMU missing: high-frequency output suspended";msg.status.push_back(status);diagnostics_pub->publish(msg);
    });
    executor.add_node(io);io_thread=std::thread([this]{executor.spin();});
  }
  ~Impl(){executor.cancel();if(io_thread.joinable())io_thread.join();backend.reset();}
  void enqueue(Auxiliary e,Audit& a){double t=eventTime(e);if(!std::isfinite(t)){++a.rejected;return;}if(t<=last_processed){++a.late;return;}if(events.size()>=max_events){++a.overflow;return;}if(!observed_ids.insert({e.index(),std::llround(t*1e9)}).second){++a.rejected;return;}auto it=std::upper_bound(events.begin(),events.end(),t,[](double t,const Auxiliary& other){return t<eventTime(other);});events.insert(it,std::move(e));}
  void ensureBackend(){if(!backend&&backend_enabled){backend_config.camera=camera;backend=std::make_unique<Backend>(backend_config,[this](const BackendResult& result){
    tf->sendTransform(transformMessage(result.map_T_odom,result.local.time,"map","odom"));State global=result.local;global.pose=result.map_T_odom*global.pose;global.velocity=result.map_T_odom.linear()*global.velocity;
    Cov transport=Cov::Identity();transport.block<3,3>(0,0)=result.map_T_odom.linear();transport.block<3,3>(12,12)=result.map_T_odom.linear();global.covariance=transport*global.covariance*transport.transpose();auto global_message=odometry(global,imu_T_base,"map",global.angular_rate,gyro_measurement_variance);
    // The graph has no velocity variables or cross covariance with the local
    // filter. Publish its pose marginal separately from the local twist marginal.
    Mat6 graph_pose=basePoseCovariance(global.pose,imu_T_base,result.covariance);
    for(int i=0;i<6;++i)for(int j=0;j<6;++j)global_message.pose.covariance[6*i+j]=graph_pose(i,j);
    global_pub->publish(global_message);
    nav_msgs::msg::Path path;path.header.frame_id="map";path.header.stamp=stamp_from_seconds(result.local.time);for(auto& pair:result.trajectory){geometry_msgs::msg::PoseStamped p;p.header=path.header;p.header.stamp=stamp_from_seconds(pair.first);p.pose=poseMessage(pair.second*imu_T_base);path.poses.push_back(p);}path_pub->publish(path);
    if(result.map){sensor_msgs::msg::PointCloud2 msg;pcl::toROSMsg(*result.map,msg);msg.header=path.header;map_pub->publish(msg);}
  });backend->reset(session);}}
  cv::Mat rectify(const sensor_msgs::msg::Image::ConstSharedPtr& image,const sensor_msgs::msg::CameraInfo& info){
    if(image->width!=info.width||image->height!=info.height)throw std::invalid_argument("Image and CameraInfo dimensions differ");
    cv::Mat K(3,3,CV_64F,const_cast<double*>(info.k.data())),R(3,3,CV_64F,const_cast<double*>(info.r.data())),P(3,4,CV_64F,const_cast<double*>(info.p.data()));
    cv::Mat D(info.d,true),output;cv::Size size(image->width,image->height);
    auto& mapx=(&info==left_info.get())?left_mapx:right_mapx;
    auto& mapy=(&info==left_info.get())?left_mapy:right_mapy;
    if(mapx.empty()) {
    if(info.distortion_model=="equidistant")cv::fisheye::initUndistortRectifyMap(K,D,R,P.colRange(0,3),size,CV_32FC1,mapx,mapy);
    else if(info.distortion_model=="plumb_bob"||info.distortion_model=="rational_polynomial"||info.distortion_model.empty())cv::initUndistortRectifyMap(K,D,R,P.colRange(0,3),size,CV_32FC1,mapx,mapy);
    else throw std::invalid_argument("Unsupported camera distortion model");
    }
    cv::remap(cv_bridge::toCvShare(image,"mono8")->image,output,mapx,mapy,cv::INTER_LINEAR);return output;
  }
};
Runtime::Runtime(rclcpp::Node::SharedPtr node):impl_(new Impl(*this,node)){
  impl_->save_service=node->create_service<std_srvs::srv::Trigger>("~/save",[this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,std::shared_ptr<std_srvs::srv::Trigger::Response> response){
    if(!impl_->backend){response->success=false;response->message="No backend state available";return;}
    response->success=impl_->backend->save(response->message);if(response->success)response->message=impl_->backend_config.output_directory;
  });
}
Runtime::~Runtime()=default;
bool Runtime::frontend()const{return impl_->side==Side::Frontend;}
bool Runtime::reset_requested()const{return impl_->reset_needed;}
void Runtime::request_reset(){impl_->reset_needed=true;}
void Runtime::reset(const Cov& P){std::lock_guard<std::mutex>l(impl_->mutex);++impl_->session;impl_->events.clear();impl_->observed_ids.clear();impl_->left_images.clear();impl_->right_images.clear();impl_->geodesy.reset();impl_->alignment.reset();impl_->tracker.reset();impl_->last_processed=-1;impl_->last_imu=-1;impl_->last_imu_wall=0;impl_->fast_time=-1;filter.reset(P);degeneracy.reset();propagator.reset();if(impl_->backend)impl_->backend->reset(impl_->session);impl_->reset_needed=false;}
void Runtime::noise(const Noise& n,double scale){propagator.configure(n,scale);}
void Runtime::process(double end,const std::function<void(double)>& predict,const std::function<State()>& get,const std::function<void(const Correction&)>& apply,const std::function<Vec3()>& angular){
  std::deque<Auxiliary> ready;{std::lock_guard<std::mutex>l(impl_->mutex);while(!impl_->events.empty()&&eventTime(impl_->events.front())<=end){ready.push_back(std::move(impl_->events.front()));impl_->events.pop_front();}}
  for(auto& event:ready){
    double time=eventTime(event);if(time<get().time-1e-8){std::lock_guard<std::mutex>l(impl_->mutex);std::visit([&](const auto& value){using T=std::decay_t<decltype(value)>;if constexpr(std::is_same_v<T,Wheel>)++impl_->wheel_audit.late;else if constexpr(std::is_same_v<T,Fix>)++impl_->gnss_audit.late;else ++impl_->stereo_audit.late;},event);continue;}
    predict(time);State state=get();BackendEvent output;output.state=state;output.session=impl_->session;
    if(auto wheel=std::get_if<Wheel>(&event)){
      bool accepted=true;
      if(frontend()){auto correction=filter.update(impl_->wheel_model.linearize(state,*wheel,angular()));if(correction)apply(*correction);else accepted=false;}else output.wheel=*wheel;
      std::lock_guard<std::mutex>l(impl_->mutex);if(accepted)++impl_->wheel_audit.accepted;else ++impl_->wheel_audit.rejected;
    }else if(auto fix=std::get_if<Fix>(&event)){
      auto aligned=impl_->alignment.align(*fix,state,impl_->antenna);bool accepted=bool(aligned);
      if(aligned){if(frontend()){auto correction=filter.update(positionObservation(state,*aligned,impl_->antenna));if(correction)apply(*correction);else accepted=false;}else output.fix=*aligned;}
      std::lock_guard<std::mutex>l(impl_->mutex);if(accepted)++impl_->gnss_audit.accepted;else ++impl_->gnss_audit.rejected;
    }else if(auto pair=std::get_if<ImagePair>(&event)){
      sensor_msgs::msg::CameraInfo::ConstSharedPtr left,right;{std::lock_guard<std::mutex>l(impl_->mutex);left=impl_->left_info;right=impl_->right_info;}
      try{
        if(!left||!right)throw std::runtime_error("Stereo CameraInfo not available");
        impl_->camera.fx=left->p[0];impl_->camera.fy=left->p[5];impl_->camera.cx=left->p[2];impl_->camera.cy=left->p[6];impl_->camera.baseline=-right->p[3]/right->p[0]+left->p[3]/left->p[0];
        if(std::abs(left->p[3])>1e-8||std::abs(left->p[5]-right->p[5])>1e-3||std::abs(left->p[6]-right->p[6])>1e-3)throw std::runtime_error("Stereo requires a rectified pair with common vertical projection and zero left Tx");
        Mat3 rectification;for(int i=0;i<3;++i)for(int j=0;j<3;++j)rectification(i,j)=left->r[3*i+j];
        impl_->camera.body_T_camera=impl_->camera_raw;impl_->camera.body_T_camera.linear()*=rectification.transpose();
        if(!impl_->camera.valid())throw std::runtime_error("Invalid stereo rectified projection/baseline");
        auto frame=impl_->tracker.track(time,impl_->rectify(pair->left,*left),impl_->rectify(pair->right,*right));
        impl_->stereo_features=frame.pixels.size();if(frontend()){auto correction=filter.stereo(frame,state,impl_->camera);if(correction){apply(*correction);++impl_->stereo_updates;}}else {output.stereo=std::move(frame);output.camera=impl_->camera;}
        std::lock_guard<std::mutex>l(impl_->mutex);++impl_->stereo_audit.accepted;
      }catch(const std::exception& e){std::lock_guard<std::mutex>l(impl_->mutex);++impl_->stereo_audit.rejected;RCLCPP_WARN_THROTTLE(impl_->io->get_logger(),*impl_->io->get_clock(),5000,"Stereo rejected: %s",e.what());}
    }
    if(!frontend()&&(output.wheel||output.fix||output.stereo)){impl_->ensureBackend();if(impl_->backend&&!impl_->backend->submit(std::move(output)))RCLCPP_WARN_THROTTLE(impl_->io->get_logger(),*impl_->io->get_clock(),1000,"Backend queue rejected observation");}
  }
  predict(end);{std::lock_guard<std::mutex>l(impl_->mutex);impl_->last_processed=end;for(auto it=impl_->observed_ids.begin();it!=impl_->observed_ids.end();)if(it->second<=std::llround(end*1e9))it=impl_->observed_ids.erase(it);else ++it;}
}
void Runtime::corrected(const State& state,const Cloud::ConstPtr& cloud){
  propagator.correct(state);impl_->local_pub->publish(odometry(state,impl_->imu_T_base,"odom",state.angular_rate,impl_->gyro_measurement_variance));
  impl_->ensureBackend();if(impl_->backend){BackendEvent event;event.state=state;event.cloud=cloud;event.degeneracy=degeneracy.status();event.session=impl_->session;impl_->backend->submit(std::move(event));}
}
void Runtime::diagnostics(const DegeneracyStatus& s,size_t lidar_queue,size_t imu_queue,double ms){
  diagnostic_msgs::msg::DiagnosticArray msg;msg.header.stamp=impl_->io->now();diagnostic_msgs::msg::DiagnosticStatus d;d.name="fastlio/estimation";d.hardware_id="fastlio";d.level=s.severe?2:s.degraded?1:0;d.message=s.severe?"lidar matching failed":s.degraded?"geometry degraded":"tracking";
  auto add=[&](const std::string& key,auto value){diagnostic_msgs::msg::KeyValue pair;pair.key=key;pair.value=std::to_string(value);d.values.push_back(pair);};
  add("session",impl_->session);add("stereo.updates",impl_->stereo_updates);add("stereo.features",impl_->stereo_features);add("degeneracy.reason",int(s.reason));add("degeneracy.severe",s.severe);add("degeneracy.degraded",s.degraded);add("weak_dimensions",s.weak_dimensions);add("match_ratio",s.match_ratio);add("residual",s.residual);add("lidar_queue",lidar_queue);add("imu_queue",imu_queue);add("processing_ms",ms);add("imu_gap_count",propagator.gaps());
  {std::lock_guard<std::mutex>l(impl_->mutex);for(auto pair:{std::make_pair("wheel",impl_->wheel_audit),std::make_pair("gnss",impl_->gnss_audit),std::make_pair("stereo",impl_->stereo_audit)}){add(std::string(pair.first)+".accepted",pair.second.accepted);add(std::string(pair.first)+".rejected",pair.second.rejected);add(std::string(pair.first)+".late",pair.second.late);add(std::string(pair.first)+".overflow",pair.second.overflow);}add("imu.invalid",impl_->invalid_imu);add("aux_queue",impl_->events.size());add("state_lag_ms",impl_->last_processed<0?0.:std::max(0.,1000*(impl_->io->now().seconds()-impl_->last_processed)));}
  if(impl_->fast_time>=0&&impl_->io->get_clock()->get_clock_type()==RCL_ROS_TIME&&impl_->io->now().seconds()-impl_->fast_time>propagator.max_gap){d.level=2;d.message="IMU missing";}
  if(impl_->backend){auto b=impl_->backend->snapshot();add("backend.dropped",b.dropped);add("backend.processed",b.processed);add("backend.loops",b.loops);add("backend.wheel_factors",b.wheel_factors);add("backend.gnss_factors",b.gnss_factors);add("backend.visual_factors",b.visual_factors);if(!b.error.empty()){d.level=2;d.message=b.error;}}
  msg.status.push_back(d);impl_->diagnostics_pub->publish(msg);
}
void Runtime::finish(){if(impl_->backend){std::string error;if(!impl_->backend->save(error))RCLCPP_ERROR(impl_->io->get_logger(),"Export failed: %s",error.c_str());}}
}
