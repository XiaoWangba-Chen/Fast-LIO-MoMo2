// This is an advanced implementation of the algorithm described in the
// following paper:
//   J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time.
//     Robotics: Science and Systems Conference (RSS). Berkeley, CA, July 2014.

// Modifier: Livox               dev@livoxtech.com

// Copyright 2013, Ji Zhang, Carnegie Mellon University
// Further contributions copyright (c) 2016, Southwest Research Institute
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from this
//    software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
#include <omp.h>
#include <mutex>
#include <math.h>
#include <thread>
#include <fstream>
#include <csignal>
#include <unistd.h>
#include <so3_math.h>
#include <fast_lio/ros2_support.hpp>
#include <Eigen/Core>
#include "IMU_Processing.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#ifdef FASTLIO_LIVOX
#include <livox_ros_driver2/msg/custom_msg.hpp>
#endif
#include "preprocess.h"
#include <ikd-Tree/ikd_Tree.h>
#include "fast_lio/runtime.hpp"
#include "fast_lio/scan_time.hpp"

#define INIT_TIME           (0.1)
#define LASER_POINT_COV     (0.001)
#define MAXN                (720000)
#define PUBFRAME_PERIOD     (20)

/*** Time Log Variables ***/
double kdtree_incremental_time = 0.0, kdtree_search_time = 0.0, kdtree_delete_time = 0.0;
double T1[MAXN], s_plot[MAXN], s_plot2[MAXN], s_plot3[MAXN], s_plot4[MAXN], s_plot5[MAXN], s_plot6[MAXN], s_plot7[MAXN], s_plot8[MAXN], s_plot9[MAXN], s_plot10[MAXN], s_plot11[MAXN];
double match_time = 0, solve_time = 0, solve_const_H_time = 0;
int    kdtree_size_st = 0, kdtree_size_end = 0, add_point_size = 0, kdtree_delete_counter = 0;
bool   runtime_pos_log = false, pcd_save_en = false, time_sync_en = false, extrinsic_est_en = true, path_en = true;
/**************************/

float res_last[100000] = {0.0};
float DET_RANGE = 300.0f;
const float MOV_THRESHOLD = 1.5f;
double time_diff_lidar_to_imu = 0.0;

mutex mtx_buffer;
condition_variable sig_buffer;

string root_dir = ROOT_DIR;
string map_file_path, lid_topic, imu_topic;

double res_mean_last = 0.05, total_residual = 0.0;
double last_timestamp_lidar = 0, last_timestamp_imu = -1.0;
double gyr_cov = 0.1, acc_cov = 0.1, b_gyr_cov = 0.0001, b_acc_cov = 0.0001;
double filter_size_corner_min = 0, filter_size_surf_min = 0, filter_size_map_min = 0, fov_deg = 0;
double cube_len = 0, HALF_FOV_COS = 0, FOV_DEG = 0, total_distance = 0, lidar_end_time = 0, first_lidar_time = 0.0;
int    effct_feat_num = 0, time_log_counter = 0, scan_count = 0, publish_count = 0;
int    iterCount = 0, feats_down_size = 0, NUM_MAX_ITERATIONS = 0, laserCloudValidNum = 0, pcd_save_interval = -1, pcd_index = 0;
bool   point_selected_surf[100000] = {0};
bool   lidar_pushed, flg_first_scan = true, flg_EKF_inited;
volatile std::sig_atomic_t flg_exit = 0;
bool   scan_pub_en = false, dense_pub_en = false, scan_body_pub_en = false;
int lidar_type;

vector<vector<int>>  pointSearchInd_surf;
vector<BoxPointType> cub_needrm;
vector<PointVector>  Nearest_Points;
vector<double>       extrinT(3, 0.0);
vector<double>       extrinR(9, 0.0);
deque<double>                     time_buffer;
deque<PointCloudXYZI::Ptr>        lidar_buffer;
deque<sensor_msgs::msg::Imu::ConstSharedPtr> imu_buffer;

PointCloudXYZI::Ptr featsFromMap(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_undistort(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_down_body(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_down_world(new PointCloudXYZI());
PointCloudXYZI::Ptr normvec(new PointCloudXYZI(100000, 1));
PointCloudXYZI::Ptr laserCloudOri(new PointCloudXYZI(100000, 1));
PointCloudXYZI::Ptr corr_normvect(new PointCloudXYZI(100000, 1));
PointCloudXYZI::Ptr _featsArray;

pcl::VoxelGrid<PointType> downSizeFilterSurf;
pcl::VoxelGrid<PointType> downSizeFilterMap;

KD_TREE<PointType> ikdtree;

V3F XAxisPoint_body(LIDAR_SP_LEN, 0.0, 0.0);
V3F XAxisPoint_world(LIDAR_SP_LEN, 0.0, 0.0);
V3D euler_cur;
V3D position_last(Zero3d);
V3D Lidar_T_wrt_IMU(Zero3d);
M3D Lidar_R_wrt_IMU(Eye3d);

/*** EKF inputs and output ***/
MeasureGroup Measures;
esekfom::esekf<state_ikfom, 12, input_ikfom> kf;
state_ikfom state_point;
vect3 pos_lid;

nav_msgs::msg::Path path;
nav_msgs::msg::Odometry odomAftMapped;
geometry_msgs::msg::Quaternion geoQuat;
geometry_msgs::msg::PoseStamped msg_body_pose;

rclcpp::Node::SharedPtr node;
std::unique_ptr<fastlio::Runtime> fusion_runtime;
double filter_time = -1;
uint64_t state_version = 0;
deque<sensor_msgs::msg::Imu::ConstSharedPtr> integration_imu;
fastlio::Vec3 filter_angular = fastlio::Vec3::Zero();
shared_ptr<Preprocess> p_pre(new Preprocess());
shared_ptr<ImuProcess> p_imu(new ImuProcess());

void SigHandle(int sig) { (void)sig;flg_exit=1; }

inline void dump_lio_state_to_log(FILE *fp)
{
    V3D rot_ang(Log(state_point.rot.toRotationMatrix()));
    fprintf(fp, "%lf ", Measures.lidar_beg_time - first_lidar_time);
    fprintf(fp, "%lf %lf %lf ", rot_ang(0), rot_ang(1), rot_ang(2));                   // Angle
    fprintf(fp, "%lf %lf %lf ", state_point.pos(0), state_point.pos(1), state_point.pos(2)); // Pos
    fprintf(fp, "%lf %lf %lf ", 0.0, 0.0, 0.0);                                        // omega
    fprintf(fp, "%lf %lf %lf ", state_point.vel(0), state_point.vel(1), state_point.vel(2)); // Vel
    fprintf(fp, "%lf %lf %lf ", 0.0, 0.0, 0.0);                                        // Acc
    fprintf(fp, "%lf %lf %lf ", state_point.bg(0), state_point.bg(1), state_point.bg(2));    // Bias_g
    fprintf(fp, "%lf %lf %lf ", state_point.ba(0), state_point.ba(1), state_point.ba(2));    // Bias_a
    fprintf(fp, "%lf %lf %lf ", state_point.grav[0], state_point.grav[1], state_point.grav[2]); // Bias_a
    fprintf(fp, "\r\n");
    fflush(fp);
}

void pointBodyToWorld_ikfom(PointType const * const pi, PointType * const po, state_ikfom &s)
{
    V3D p_body(pi->x, pi->y, pi->z);
    V3D p_global(s.rot * (s.offset_R_L_I*p_body + s.offset_T_L_I) + s.pos);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
}


void pointBodyToWorld(PointType const * const pi, PointType * const po)
{
    V3D p_body(pi->x, pi->y, pi->z);
    V3D p_global(state_point.rot * (state_point.offset_R_L_I*p_body + state_point.offset_T_L_I) + state_point.pos);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
}

template<typename T>
void pointBodyToWorld(const Matrix<T, 3, 1> &pi, Matrix<T, 3, 1> &po)
{
    V3D p_body(pi[0], pi[1], pi[2]);
    V3D p_global(state_point.rot * (state_point.offset_R_L_I*p_body + state_point.offset_T_L_I) + state_point.pos);

    po[0] = p_global(0);
    po[1] = p_global(1);
    po[2] = p_global(2);
}

void RGBpointBodyToWorld(PointType const * const pi, PointType * const po)
{
    V3D p_body(pi->x, pi->y, pi->z);
    V3D p_global(state_point.rot * (state_point.offset_R_L_I*p_body + state_point.offset_T_L_I) + state_point.pos);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
}

void RGBpointBodyLidarToIMU(PointType const * const pi, PointType * const po)
{
    V3D p_body_lidar(pi->x, pi->y, pi->z);
    V3D p_body_imu(state_point.offset_R_L_I*p_body_lidar + state_point.offset_T_L_I);

    po->x = p_body_imu(0);
    po->y = p_body_imu(1);
    po->z = p_body_imu(2);
    po->intensity = pi->intensity;
}

void points_cache_collect()
{
    PointVector points_history;
    ikdtree.acquire_removed_points(points_history);
    // for (int i = 0; i < points_history.size(); i++) _featsArray->push_back(points_history[i]);
}

BoxPointType LocalMap_Points;
bool Localmap_Initialized = false;
void lasermap_fov_segment()
{
    cub_needrm.clear();
    kdtree_delete_counter = 0;
    kdtree_delete_time = 0.0;
    pointBodyToWorld(XAxisPoint_body, XAxisPoint_world);
    V3D pos_LiD = pos_lid;
    if (!Localmap_Initialized){
        for (int i = 0; i < 3; i++){
            LocalMap_Points.vertex_min[i] = pos_LiD(i) - cube_len / 2.0;
            LocalMap_Points.vertex_max[i] = pos_LiD(i) + cube_len / 2.0;
        }
        Localmap_Initialized = true;
        return;
    }
    float dist_to_map_edge[3][2];
    bool need_move = false;
    for (int i = 0; i < 3; i++){
        dist_to_map_edge[i][0] = fabs(pos_LiD(i) - LocalMap_Points.vertex_min[i]);
        dist_to_map_edge[i][1] = fabs(pos_LiD(i) - LocalMap_Points.vertex_max[i]);
        if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE || dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE) need_move = true;
    }
    if (!need_move) return;
    BoxPointType New_LocalMap_Points, tmp_boxpoints;
    New_LocalMap_Points = LocalMap_Points;
    float mov_dist = max((cube_len - 2.0 * MOV_THRESHOLD * DET_RANGE) * 0.5 * 0.9, double(DET_RANGE * (MOV_THRESHOLD -1)));
    for (int i = 0; i < 3; i++){
        tmp_boxpoints = LocalMap_Points;
        if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE){
            New_LocalMap_Points.vertex_max[i] -= mov_dist;
            New_LocalMap_Points.vertex_min[i] -= mov_dist;
            tmp_boxpoints.vertex_min[i] = LocalMap_Points.vertex_max[i] - mov_dist;
            cub_needrm.push_back(tmp_boxpoints);
        } else if (dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE){
            New_LocalMap_Points.vertex_max[i] += mov_dist;
            New_LocalMap_Points.vertex_min[i] += mov_dist;
            tmp_boxpoints.vertex_max[i] = LocalMap_Points.vertex_min[i] + mov_dist;
            cub_needrm.push_back(tmp_boxpoints);
        }
    }
    LocalMap_Points = New_LocalMap_Points;

    points_cache_collect();
    double delete_begin = omp_get_wtime();
    if(cub_needrm.size() > 0) kdtree_delete_counter = ikdtree.Delete_Point_Boxes(cub_needrm);
    kdtree_delete_time = omp_get_wtime() - delete_begin;
}

void standard_pcl_cbk(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg)
{
    mtx_buffer.lock();
    scan_count ++;
    double preprocess_start_time = omp_get_wtime();
    if (stamp_seconds(msg->header.stamp) < last_timestamp_lidar)
    {
        RCLCPP_ERROR(rclcpp::get_logger("fast_lio"), "lidar loop back, clear buffer");
        if (fusion_runtime) fusion_runtime->request_reset();
        lidar_buffer.clear();
        time_buffer.clear();
    }

    PointCloudXYZI::Ptr  ptr(new PointCloudXYZI());
    p_pre->process(msg, ptr);
    ptr->erase(std::remove_if(ptr->begin(),ptr->end(),[](const PointType& p){return !std::isfinite(p.x)||!std::isfinite(p.y)||!std::isfinite(p.z)||!std::isfinite(p.curvature)||p.curvature<0||p.curvature>1000;}),ptr->end());
    if(ptr->size()>99000) { ptr->resize(99000); }
    std::sort(ptr->begin(),ptr->end(),[](const PointType& a,const PointType& b){return a.curvature<b.curvature;});
    if(lidar_buffer.size()>=100) { mtx_buffer.unlock(); fusion_runtime->request_reset(); return; }
    lidar_buffer.push_back(ptr);
    time_buffer.push_back(stamp_seconds(msg->header.stamp));
    last_timestamp_lidar = stamp_seconds(msg->header.stamp);
    s_plot11[scan_count % MAXN] = omp_get_wtime() - preprocess_start_time;
    mtx_buffer.unlock();
    sig_buffer.notify_all();
}

double timediff_lidar_wrt_imu = 0.0;
bool   timediff_set_flg = false;
#ifdef FASTLIO_LIVOX
void livox_pcl_cbk(const livox_ros_driver2::msg::CustomMsg::ConstSharedPtr &msg)
{
    mtx_buffer.lock();
    double preprocess_start_time = omp_get_wtime();
    scan_count ++;
    if (stamp_seconds(msg->header.stamp) < last_timestamp_lidar)
    {
        RCLCPP_ERROR(rclcpp::get_logger("fast_lio"), "lidar loop back, clear buffer");
        if (fusion_runtime) fusion_runtime->request_reset();
        lidar_buffer.clear();
        time_buffer.clear();
    }
    last_timestamp_lidar = stamp_seconds(msg->header.stamp);

    if (!time_sync_en && abs(last_timestamp_imu - last_timestamp_lidar) > 10.0 && !imu_buffer.empty() && !lidar_buffer.empty() )
    {
        printf("IMU and LiDAR not Synced, IMU time: %lf, lidar header time: %lf \n",last_timestamp_imu, last_timestamp_lidar);
    }

    if (time_sync_en && !timediff_set_flg && abs(last_timestamp_lidar - last_timestamp_imu) > 1 && !imu_buffer.empty())
    {
        timediff_set_flg = true;
        timediff_lidar_wrt_imu = last_timestamp_lidar + 0.1 - last_timestamp_imu;
        printf("Self sync IMU and LiDAR, time diff is %.10lf \n", timediff_lidar_wrt_imu);
    }

    PointCloudXYZI::Ptr  ptr(new PointCloudXYZI());
    p_pre->process(msg, ptr);
    ptr->erase(std::remove_if(ptr->begin(),ptr->end(),[](const PointType& p){return !std::isfinite(p.x)||!std::isfinite(p.y)||!std::isfinite(p.z)||!std::isfinite(p.curvature)||p.curvature<0||p.curvature>1000;}),ptr->end());
    if(ptr->size()>99000) { ptr->resize(99000); }
    std::sort(ptr->begin(),ptr->end(),[](const PointType& a,const PointType& b){return a.curvature<b.curvature;});
    if(lidar_buffer.size()>=100) { mtx_buffer.unlock(); fusion_runtime->request_reset(); return; }
    lidar_buffer.push_back(ptr);
    time_buffer.push_back(last_timestamp_lidar);

    s_plot11[scan_count % MAXN] = omp_get_wtime() - preprocess_start_time;
    mtx_buffer.unlock();
    sig_buffer.notify_all();
}

#endif
void imu_cbk(const sensor_msgs::msg::Imu::ConstSharedPtr &msg_in)
{
    const auto& a=msg_in->linear_acceleration;const auto& w=msg_in->angular_velocity;
    if(!std::isfinite(a.x)||!std::isfinite(a.y)||!std::isfinite(a.z)||!std::isfinite(w.x)||!std::isfinite(w.y)||!std::isfinite(w.z)){
        RCLCPP_WARN_THROTTLE(node->get_logger(),*node->get_clock(),1000,"Rejecting non-finite IMU sample");return;
    }
    if(!p_imu->initialized()&&a.x*a.x+a.y*a.y+a.z*a.z<1e-6)return;
    if(stamp_seconds(msg_in->header.stamp)-time_diff_lidar_to_imu<0)return;
    publish_count ++;
    // cout<<"IMU got at: "<<stamp_seconds(msg_in->header.stamp)<<endl;
    sensor_msgs::msg::Imu::SharedPtr msg(new sensor_msgs::msg::Imu(*msg_in));

    msg->header.stamp = stamp_from_seconds(stamp_seconds(msg_in->header.stamp) - time_diff_lidar_to_imu);
    if (abs(timediff_lidar_wrt_imu) > 0.1 && time_sync_en)
    {
        msg->header.stamp = \
        stamp_from_seconds(timediff_lidar_wrt_imu + stamp_seconds(msg_in->header.stamp));
    }

    double timestamp = stamp_seconds(msg->header.stamp);

    mtx_buffer.lock();

    if (timestamp < last_timestamp_imu)
    {
        RCLCPP_WARN(rclcpp::get_logger("fast_lio"), "imu loop back, clear buffer");
        if (fusion_runtime) fusion_runtime->request_reset();
        imu_buffer.clear();
    }

    last_timestamp_imu = timestamp;

    if (!integration_imu.empty() && timestamp == stamp_seconds(integration_imu.back()->header.stamp)) { mtx_buffer.unlock(); return; }
    imu_buffer.push_back(msg);
    integration_imu.push_back(msg);
    if (imu_buffer.size()>10000 || integration_imu.size()>10000) fusion_runtime->request_reset();
    mtx_buffer.unlock();
    sig_buffer.notify_all();
}

double lidar_mean_scantime = 0.0;
int    scan_num = 0;
bool sync_packages(MeasureGroup &meas)
{
    if (lidar_buffer.empty() || imu_buffer.empty()) {
        return false;
    }

    /*** push a lidar scan ***/
    if(!lidar_pushed)
    {
        meas.lidar = lidar_buffer.front();
        meas.lidar_beg_time = time_buffer.front();


        if (meas.lidar->points.size() <= 1) // time too little
        {
            lidar_end_time = meas.lidar_beg_time + lidar_mean_scantime;
            RCLCPP_WARN(rclcpp::get_logger("fast_lio"), "Too few input point cloud!\n");
        }
        else if (meas.lidar->points.back().curvature / double(1000) < 0.5 * lidar_mean_scantime)
        {
            lidar_end_time = meas.lidar_beg_time + lidar_mean_scantime;
        }
        else
        {
            scan_num ++;
            lidar_end_time = meas.lidar_beg_time + meas.lidar->points.back().curvature / double(1000);
            lidar_mean_scantime += (meas.lidar->points.back().curvature / double(1000) - lidar_mean_scantime) / scan_num;
        }
        if(lidar_type == MARSIM)
            lidar_end_time = meas.lidar_beg_time;

        meas.lidar_end_time = lidar_end_time;

        lidar_pushed = true;
    }

    if (last_timestamp_imu < lidar_end_time)
    {
        return false;
    }

    /*** push imu data, and pop from imu buffer ***/
    double imu_time = stamp_seconds(imu_buffer.front()->header.stamp);
    meas.imu.clear();
    while ((!imu_buffer.empty()) && (imu_time < lidar_end_time))
    {
        imu_time = stamp_seconds(imu_buffer.front()->header.stamp);
        if(imu_time > lidar_end_time) break;
        meas.imu.push_back(imu_buffer.front());
        imu_buffer.pop_front();
    }

    if (!imu_buffer.empty()) meas.imu.push_back(imu_buffer.front());
    if(meas.imu.empty()) return false;
    lidar_buffer.pop_front();
    time_buffer.pop_front();
    lidar_pushed = false;
    return true;
}

int process_increments = 0;
void map_incremental()
{
    PointVector PointToAdd;
    PointVector PointNoNeedDownsample;
    PointToAdd.reserve(feats_down_size);
    PointNoNeedDownsample.reserve(feats_down_size);
    for (int i = 0; i < feats_down_size; i++)
    {
        /* transform to world frame */
        pointBodyToWorld(&(feats_down_body->points[i]), &(feats_down_world->points[i]));
        /* decide if need add to map */
        if (!Nearest_Points[i].empty() && flg_EKF_inited)
        {
            const PointVector &points_near = Nearest_Points[i];
            bool need_add = true;
            BoxPointType Box_of_Point;
            PointType downsample_result, mid_point;
            mid_point.x = floor(feats_down_world->points[i].x/filter_size_map_min)*filter_size_map_min + 0.5 * filter_size_map_min;
            mid_point.y = floor(feats_down_world->points[i].y/filter_size_map_min)*filter_size_map_min + 0.5 * filter_size_map_min;
            mid_point.z = floor(feats_down_world->points[i].z/filter_size_map_min)*filter_size_map_min + 0.5 * filter_size_map_min;
            float dist  = calc_dist(feats_down_world->points[i],mid_point);
            if (fabs(points_near[0].x - mid_point.x) > 0.5 * filter_size_map_min && fabs(points_near[0].y - mid_point.y) > 0.5 * filter_size_map_min && fabs(points_near[0].z - mid_point.z) > 0.5 * filter_size_map_min){
                PointNoNeedDownsample.push_back(feats_down_world->points[i]);
                continue;
            }
            for (int readd_i = 0; readd_i < NUM_MATCH_POINTS; readd_i ++)
            {
                if (points_near.size() < NUM_MATCH_POINTS) break;
                if (calc_dist(points_near[readd_i], mid_point) < dist)
                {
                    need_add = false;
                    break;
                }
            }
            if (need_add) PointToAdd.push_back(feats_down_world->points[i]);
        }
        else
        {
            PointToAdd.push_back(feats_down_world->points[i]);
        }
    }

    double st_time = omp_get_wtime();
    add_point_size = ikdtree.Add_Points(PointToAdd, true);
    ikdtree.Add_Points(PointNoNeedDownsample, false);
    add_point_size = PointToAdd.size() + PointNoNeedDownsample.size();
    kdtree_incremental_time = omp_get_wtime() - st_time;
}

PointCloudXYZI::Ptr pcl_wait_pub(new PointCloudXYZI(500000, 1));
PointCloudXYZI::Ptr pcl_wait_save(new PointCloudXYZI());
void publish_frame_world(const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr & pubLaserCloudFull)
{
    if(scan_pub_en)
    {
        PointCloudXYZI::Ptr laserCloudFullRes(dense_pub_en ? feats_undistort : feats_down_body);
        int size = laserCloudFullRes->points.size();
        PointCloudXYZI::Ptr laserCloudWorld( \
                        new PointCloudXYZI(size, 1));

        for (int i = 0; i < size; i++)
        {
            RGBpointBodyToWorld(&laserCloudFullRes->points[i], \
                                &laserCloudWorld->points[i]);
        }

        sensor_msgs::msg::PointCloud2 laserCloudmsg;
        pcl::toROSMsg(*laserCloudWorld, laserCloudmsg);
        laserCloudmsg.header.stamp = stamp_from_seconds(lidar_end_time);
        laserCloudmsg.header.frame_id = "odom";
        pubLaserCloudFull->publish(laserCloudmsg);
        publish_count -= PUBFRAME_PERIOD;
    }

    /**************** save map ****************/
    /* 1. make sure you have enough memories
    /* 2. noted that pcd save will influence the real-time performences **/
    if (pcd_save_en)
    {
        int size = feats_undistort->points.size();
        PointCloudXYZI::Ptr laserCloudWorld( \
                        new PointCloudXYZI(size, 1));

        for (int i = 0; i < size; i++)
        {
            RGBpointBodyToWorld(&feats_undistort->points[i], \
                                &laserCloudWorld->points[i]);
        }
        *pcl_wait_save += *laserCloudWorld;

        static int scan_wait_num = 0;
        scan_wait_num ++;
        if (pcl_wait_save->size() > 0 && pcd_save_interval > 0  && scan_wait_num >= pcd_save_interval)
        {
            pcd_index ++;
            string all_points_dir(string(string(ROOT_DIR) + "PCD/scans_") + to_string(pcd_index) + string(".pcd"));
            pcl::PCDWriter pcd_writer;
            cout << "current scan saved to /PCD/" << all_points_dir << endl;
            pcd_writer.writeBinary(all_points_dir, *pcl_wait_save);
            pcl_wait_save->clear();
            scan_wait_num = 0;
        }
    }
}

void publish_frame_body(const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr & pubLaserCloudFull_body)
{
    int size = feats_undistort->points.size();
    PointCloudXYZI::Ptr laserCloudIMUBody(new PointCloudXYZI(size, 1));

    for (int i = 0; i < size; i++)
    {
        RGBpointBodyLidarToIMU(&feats_undistort->points[i], \
                            &laserCloudIMUBody->points[i]);
    }

    sensor_msgs::msg::PointCloud2 laserCloudmsg;
    pcl::toROSMsg(*laserCloudIMUBody, laserCloudmsg);
    laserCloudmsg.header.stamp = stamp_from_seconds(lidar_end_time);
    laserCloudmsg.header.frame_id = "imu_link";
    pubLaserCloudFull_body->publish(laserCloudmsg);
    publish_count -= PUBFRAME_PERIOD;
}

void publish_effect_world(const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr & pubLaserCloudEffect)
{
    PointCloudXYZI::Ptr laserCloudWorld( \
                    new PointCloudXYZI(effct_feat_num, 1));
    for (int i = 0; i < effct_feat_num; i++)
    {
        RGBpointBodyToWorld(&laserCloudOri->points[i], \
                            &laserCloudWorld->points[i]);
    }
    sensor_msgs::msg::PointCloud2 laserCloudFullRes3;
    pcl::toROSMsg(*laserCloudWorld, laserCloudFullRes3);
    laserCloudFullRes3.header.stamp = stamp_from_seconds(lidar_end_time);
    laserCloudFullRes3.header.frame_id = "odom";
    pubLaserCloudEffect->publish(laserCloudFullRes3);
}

void publish_map(const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr & pubLaserCloudMap)
{
    sensor_msgs::msg::PointCloud2 laserCloudMap;
    pcl::toROSMsg(*featsFromMap, laserCloudMap);
    laserCloudMap.header.stamp = stamp_from_seconds(lidar_end_time);
    laserCloudMap.header.frame_id = "odom";
    pubLaserCloudMap->publish(laserCloudMap);
}

template<typename T>
void set_posestamp(T & out)
{
    out.pose.position.x = state_point.pos(0);
    out.pose.position.y = state_point.pos(1);
    out.pose.position.z = state_point.pos(2);
    out.pose.orientation.x = geoQuat.x;
    out.pose.orientation.y = geoQuat.y;
    out.pose.orientation.z = geoQuat.z;
    out.pose.orientation.w = geoQuat.w;

}

void publish_path(const rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubPath)
{
    set_posestamp(msg_body_pose);
    msg_body_pose.header.stamp = stamp_from_seconds(lidar_end_time);
    msg_body_pose.header.frame_id = "odom";

    /*** if path is too large, the rvis will crash ***/
    static int jjj = 0;
    jjj++;
    if (jjj % 10 == 0)
    {
        path.poses.push_back(msg_body_pose);
        pubPath->publish(path);
    }
}

fastlio::State exported_state() {
    auto x=kf.get_x(); fastlio::State s;
    s.time=filter_time;s.version=state_version;s.pose.linear()=x.rot.toRotationMatrix();s.pose.translation()=x.pos;
    s.velocity=x.vel;s.bg=x.bg;s.ba=x.ba;s.gravity=x.grav;s.covariance=kf.get_P();s.angular_rate=filter_angular;
    Eigen::Matrix<double,2,1> zero=Eigen::Matrix<double,2,1>::Zero();
    x.S2_Mx(s.gravity_basis,zero,21);return s;
}
fastlio::Cov reset_jacobian(const state_ikfom& before,const Eigen::Matrix<double,23,1>& dx){
    auto after=before;after.boxplus(dx);fastlio::Cov J;
    for(int i=0;i<23;++i){auto plus=before,minus=before;auto a=dx,b=dx;a[i]+=1e-6;b[i]-=1e-6;plus.boxplus(a);minus.boxplus(b);
      Eigen::Matrix<double,23,1> ep,em;plus.boxminus(ep,after);minus.boxminus(em,after);J.col(i)=(ep-em)/2e-6;}
    return J;
}
void apply_correction(const fastlio::Correction& correction) {
    auto x=kf.get_x();fastlio::Cov exact=reset_jacobian(x,correction.delta),applied=fastlio::Cov::Identity();
    for(int index:{3,6})applied.block<3,3>(index,index)=fastlio::rightJacobian(correction.delta.segment<3>(index));
    fusion_runtime->filter.transportCurrent(exact*applied.inverse());
    x.boxplus(correction.delta);fastlio::Cov P=fusion_runtime->filter.covariance().topLeftCorner<23,23>();
    kf.change_x(x);kf.change_P(P);++state_version;
}
void predict_until(double target) {
    if(target<filter_time-1e-8)throw std::runtime_error("Estimator timestamp moved backwards");
    while(filter_time<target-1e-9) {
        while(integration_imu.size()>2 && stamp_seconds(integration_imu[1]->header.stamp)<=filter_time)integration_imu.pop_front();
        if(integration_imu.size()<2)throw std::runtime_error("Missing bracketing IMU");
        const auto& a=integration_imu[0];const auto& b=integration_imu[1];
        double begin=stamp_seconds(a->header.stamp),end=stamp_seconds(b->header.stamp);
        if(begin>filter_time+1e-7 || end-begin>fusion_runtime->propagator.max_gap)throw std::runtime_error("IMU gap");
        double stop=std::min(target,end);
        double alpha=(.5*(filter_time+stop)-begin)/(end-begin);
        fastlio::Vec3 aa(a->linear_acceleration.x,a->linear_acceleration.y,a->linear_acceleration.z),ab(b->linear_acceleration.x,b->linear_acceleration.y,b->linear_acceleration.z);
        fastlio::Vec3 ga(a->angular_velocity.x,a->angular_velocity.y,a->angular_velocity.z),gb(b->angular_velocity.x,b->angular_velocity.y,b->angular_velocity.z);
        input_ikfom input;input.acc=((1-alpha)*aa+alpha*ab)*p_imu->acceleration_scale();input.gyro=(1-alpha)*ga+alpha*gb;
        double dt=stop-filter_time;if(dt<=0)throw std::runtime_error("Non-positive IMU interval");
        kf.predict(dt,p_imu->Q,input);filter_time=stop;filter_angular=(1-alpha)*ga+alpha*gb;
    }
}
void publish_prediction_only(double started) {
    fusion_runtime->degeneracy.analyze(Eigen::MatrixXd::Zero(0,6),LASER_POINT_COV,0,1.0,true);
    ++state_version;fusion_runtime->corrected(exported_state(),nullptr);
    fusion_runtime->diagnostics(fusion_runtime->degeneracy.status(),lidar_buffer.size(),imu_buffer.size(),1000*(omp_get_wtime()-started));
}
void h_share_model(state_ikfom &s, esekfom::dyn_share_datastruct<double> &ekfom_data)
{
    double match_start = omp_get_wtime();
    laserCloudOri->clear();
    corr_normvect->clear();
    total_residual = 0.0;

    /** closest surface search and residual computation **/
    #ifdef MP_EN
        omp_set_num_threads(MP_PROC_NUM);
        #pragma omp parallel for
    #endif
    for (int i = 0; i < feats_down_size; i++)
    {
        PointType &point_body  = feats_down_body->points[i];
        PointType &point_world = feats_down_world->points[i];

        /* transform to world frame */
        V3D p_body(point_body.x, point_body.y, point_body.z);
        V3D p_global(s.rot * (s.offset_R_L_I*p_body + s.offset_T_L_I) + s.pos);
        point_world.x = p_global(0);
        point_world.y = p_global(1);
        point_world.z = p_global(2);
        point_world.intensity = point_body.intensity;

        vector<float> pointSearchSqDis(NUM_MATCH_POINTS);

        auto &points_near = Nearest_Points[i];

        if (ekfom_data.converge)
        {
            /** Find the closest surfaces in the map **/
            ikdtree.Nearest_Search(point_world, NUM_MATCH_POINTS, points_near, pointSearchSqDis);
            point_selected_surf[i] = points_near.size() < NUM_MATCH_POINTS ? false : pointSearchSqDis[NUM_MATCH_POINTS - 1] > 5 ? false : true;
        }

        if (!point_selected_surf[i]) continue;

        VF(4) pabcd;
        point_selected_surf[i] = false;
        if (esti_plane(pabcd, points_near, 0.1f))
        {
            float pd2 = pabcd(0) * point_world.x + pabcd(1) * point_world.y + pabcd(2) * point_world.z + pabcd(3);
            float s = 1 - 0.9 * fabs(pd2) / sqrt(p_body.norm());

            if (s > 0.9)
            {
                point_selected_surf[i] = true;
                normvec->points[i].x = pabcd(0);
                normvec->points[i].y = pabcd(1);
                normvec->points[i].z = pabcd(2);
                normvec->points[i].intensity = pd2;
                res_last[i] = abs(pd2);
            }
        }
    }

    effct_feat_num = 0;

    for (int i = 0; i < feats_down_size; i++)
    {
        if (point_selected_surf[i])
        {
            laserCloudOri->points[effct_feat_num] = feats_down_body->points[i];
            corr_normvect->points[effct_feat_num] = normvec->points[i];
            total_residual += res_last[i];
            effct_feat_num ++;
        }
    }

    if (effct_feat_num < 1)
    {
        fastlio::DegeneracyStatus status = fusion_runtime->degeneracy.analyze(Eigen::MatrixXd::Zero(0,12),LASER_POINT_COV,feats_down_size,1.0,false);
        ekfom_data.valid = false;
        RCLCPP_WARN(rclcpp::get_logger("fast_lio"), "No Effective Points! \n");
        return;
    }

    res_mean_last = total_residual / effct_feat_num;
    match_time  += omp_get_wtime() - match_start;
    double solve_start_  = omp_get_wtime();

    /*** Computation of Measuremnt Jacobian matrix H and measurents vector ***/
    ekfom_data.h_x = MatrixXd::Zero(effct_feat_num, 12); //23
    ekfom_data.h.resize(effct_feat_num);

    for (int i = 0; i < effct_feat_num; i++)
    {
        const PointType &laser_p  = laserCloudOri->points[i];
        V3D point_this_be(laser_p.x, laser_p.y, laser_p.z);
        M3D point_be_crossmat;
        point_be_crossmat << SKEW_SYM_MATRX(point_this_be);
        V3D point_this = s.offset_R_L_I * point_this_be + s.offset_T_L_I;
        M3D point_crossmat;
        point_crossmat<<SKEW_SYM_MATRX(point_this);

        /*** get the normal vector of closest surface/corner ***/
        const PointType &norm_p = corr_normvect->points[i];
        V3D norm_vec(norm_p.x, norm_p.y, norm_p.z);

        /*** calculate the Measuremnt Jacobian matrix H ***/
        V3D C(s.rot.conjugate() *norm_vec);
        V3D A(point_crossmat * C);
        if (extrinsic_est_en)
        {
            V3D B(point_be_crossmat * s.offset_R_L_I.conjugate() * C); //s.rot.conjugate()*norm_vec);
            ekfom_data.h_x.block<1, 12>(i,0) << norm_p.x, norm_p.y, norm_p.z, VEC_FROM_ARRAY(A), VEC_FROM_ARRAY(B), VEC_FROM_ARRAY(C);
        }
        else
        {
            ekfom_data.h_x.block<1, 12>(i,0) << norm_p.x, norm_p.y, norm_p.z, VEC_FROM_ARRAY(A), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
        }

        /*** Measuremnt: distance to the closest surface/corner ***/
        ekfom_data.h(i) = -norm_p.intensity;
    }
    solve_time += omp_get_wtime() - solve_start_;
}

int run_mapping(int argc, char** argv)
{
    rclcpp::init(argc,argv,rclcpp::InitOptions(),rclcpp::SignalHandlerOptions::None);
    node = std::make_shared<rclcpp::Node>("fastlio_mapping");

    path_en = node->declare_parameter<bool>("publish.path_en", true);
    scan_pub_en = node->declare_parameter<bool>("publish.scan_publish_en", true);
    dense_pub_en = node->declare_parameter<bool>("publish.dense_publish_en", true);
    scan_body_pub_en = node->declare_parameter<bool>("publish.scan_bodyframe_pub_en", true);
    NUM_MAX_ITERATIONS = node->declare_parameter<int>("max_iteration", 4);
    map_file_path = node->declare_parameter<string>("map_file_path", "");
    lid_topic = node->declare_parameter<string>("common.lid_topic", "/livox/lidar");
    imu_topic = node->declare_parameter<string>("common.imu_topic", "/livox/imu");
    time_sync_en = node->declare_parameter<bool>("common.time_sync_en", false);
    time_diff_lidar_to_imu = node->declare_parameter<double>("common.time_offset_lidar_to_imu", 0.0);
    filter_size_corner_min = node->declare_parameter<double>("filter_size_corner", 0.5);
    filter_size_surf_min = node->declare_parameter<double>("filter_size_surf", 0.5);
    filter_size_map_min = node->declare_parameter<double>("filter_size_map", 0.5);
    cube_len = node->declare_parameter<double>("cube_side_length", 1000.0);
    DET_RANGE = node->declare_parameter<double>("mapping.det_range", 300.0);
    fov_deg = node->declare_parameter<double>("mapping.fov_degree", 180);
    gyr_cov = node->declare_parameter<double>("mapping.gyr_cov", 0.1);
    acc_cov = node->declare_parameter<double>("mapping.acc_cov", 0.1);
    b_gyr_cov = node->declare_parameter<double>("mapping.b_gyr_cov", 0.0001);
    b_acc_cov = node->declare_parameter<double>("mapping.b_acc_cov", 0.0001);
    p_pre->blind = node->declare_parameter<double>("preprocess.blind", 0.01);
    lidar_type = node->declare_parameter<int>("preprocess.lidar_type", 3);
    p_pre->N_SCANS = node->declare_parameter<int>("preprocess.scan_line", 16);
    p_pre->time_unit = node->declare_parameter<int>("preprocess.timestamp_unit", US);
    p_pre->SCAN_RATE = node->declare_parameter<int>("preprocess.scan_rate", 10);
    p_pre->point_filter_num = node->declare_parameter<int>("point_filter_num", 2);
    p_pre->feature_enabled = node->declare_parameter<bool>("feature_extract_enable", false);
    runtime_pos_log = node->declare_parameter<bool>("runtime_pos_log_enable", 0);
    extrinsic_est_en = node->declare_parameter<bool>("mapping.extrinsic_est_en", true);
    pcd_save_en = node->declare_parameter<bool>("pcd_save.pcd_save_en", false);
    pcd_save_interval = node->declare_parameter<int>("pcd_save.interval", -1);
    extrinT = node->declare_parameter<vector<double>>("mapping.extrinsic_T", vector<double>{0,0,0});
    extrinR = node->declare_parameter<vector<double>>("mapping.extrinsic_R", vector<double>{1,0,0,0,1,0,0,0,1});

    for(double variance:{gyr_cov,acc_cov,b_gyr_cov,b_acc_cov})if(!std::isfinite(variance)||variance<0)throw std::invalid_argument("IMU noise variances must be finite and nonnegative");
    if(!std::isfinite(cube_len)||!std::isfinite(DET_RANGE)||DET_RANGE<=0||cube_len<=2*MOV_THRESHOLD*DET_RANGE)throw std::invalid_argument("cube_side_length must exceed 3 * mapping.det_range to prevent map-window oscillation");
    if(extrinT.size()!=3 || extrinR.size()!=9)throw std::invalid_argument("mapping extrinsic requires T[3], R[9]");
    if(p_pre->N_SCANS<1||p_pre->N_SCANS>128||p_pre->point_filter_num<1||filter_size_surf_min<=0||filter_size_map_min<=0)throw std::invalid_argument("Invalid preprocessing configuration");
    if(time_sync_en)throw std::invalid_argument("ROS2 fusion requires calibrated timestamps; disable common.time_sync_en");
    const bool reliable_inputs=node->declare_parameter<bool>("common.reliable_inputs",false);
    fusion_runtime=std::make_unique<fastlio::Runtime>(node);
    kf.prediction_observer=[](const fastlio::Cov& F,const fastlio::Cov& P){if(fusion_runtime->frontend())fusion_runtime->filter.predict(F,P);};
    p_pre->lidar_type = lidar_type;
    cout<<"p_pre->lidar_type "<<p_pre->lidar_type<<endl;

    path.header.stamp    = node->now();
    path.header.frame_id ="odom";

    /*** variables definition ***/
    int effect_feat_num = 0, frame_num = 0;
    double deltaT, deltaR, aver_time_consu = 0, aver_time_icp = 0, aver_time_match = 0, aver_time_incre = 0, aver_time_solve = 0, aver_time_const_H_time = 0;
    bool flg_EKF_converged, EKF_stop_flg = 0;

    FOV_DEG = (fov_deg + 10.0) > 179.9 ? 179.9 : (fov_deg + 10.0);
    HALF_FOV_COS = cos((FOV_DEG) * 0.5 * PI_M / 180.0);

    _featsArray.reset(new PointCloudXYZI());

    memset(point_selected_surf, true, sizeof(point_selected_surf));
    memset(res_last, -1000.0f, sizeof(res_last));
    downSizeFilterSurf.setLeafSize(filter_size_surf_min, filter_size_surf_min, filter_size_surf_min);
    downSizeFilterMap.setLeafSize(filter_size_map_min, filter_size_map_min, filter_size_map_min);
    memset(point_selected_surf, true, sizeof(point_selected_surf));
    memset(res_last, -1000.0f, sizeof(res_last));

    Lidar_T_wrt_IMU<<VEC_FROM_ARRAY(extrinT);
    Lidar_R_wrt_IMU<<MAT_FROM_ARRAY(extrinR);
    p_imu->set_extrinsic(Lidar_T_wrt_IMU, Lidar_R_wrt_IMU);
    p_imu->set_gyr_cov(V3D(gyr_cov, gyr_cov, gyr_cov));
    p_imu->set_acc_cov(V3D(acc_cov, acc_cov, acc_cov));
    p_imu->set_gyr_bias_cov(V3D(b_gyr_cov, b_gyr_cov, b_gyr_cov));
    p_imu->set_acc_bias_cov(V3D(b_acc_cov, b_acc_cov, b_acc_cov));
    p_imu->lidar_type = lidar_type;
    double epsi[23] = {0.001};
    fill(epsi, epsi+23, 0.001);
    kf.init_dyn_share(get_f, df_dx, df_dw, h_share_model, NUM_MAX_ITERATIONS, epsi);

    /*** debug record ***/
    FILE *fp;
    string pos_log_dir = root_dir + "/Log/pos_log.txt";
    fp = fopen(pos_log_dir.c_str(),"w");

    ofstream fout_pre, fout_out, fout_dbg;
    fout_pre.open(DEBUG_FILE_DIR("mat_pre.txt"),ios::out);
    fout_out.open(DEBUG_FILE_DIR("mat_out.txt"),ios::out);
    fout_dbg.open(DEBUG_FILE_DIR("dbg.txt"),ios::out);
    if (fout_pre && fout_out)
        cout << "~~~~"<<ROOT_DIR<<" file opened" << endl;
    else
        cout << "~~~~"<<ROOT_DIR<<" doesn't exist" << endl;

    /*** ROS subscribe initialization ***/
    auto cloud_qos=rclcpp::SensorDataQoS().keep_last(20);
    if(reliable_inputs)cloud_qos.reliable();
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_pcl;
#ifdef FASTLIO_LIVOX
    rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr sub_livox;
    if (lidar_type == AVIA) sub_livox = node->create_subscription<livox_ros_driver2::msg::CustomMsg>(lid_topic, cloud_qos, livox_pcl_cbk);
    else
#else
    if (lidar_type == AVIA) throw std::runtime_error("Livox support disabled; select lidar_type 2/3/4 or build with FASTLIO_LIVOX=ON");
#endif
    sub_pcl = node->create_subscription<sensor_msgs::msg::PointCloud2>(lid_topic, cloud_qos, standard_pcl_cbk);
    auto sub_imu = node->create_subscription<sensor_msgs::msg::Imu>(imu_topic, rclcpp::SensorDataQoS().keep_last(2000), imu_cbk);
    auto pubLaserCloudFull = node->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_registered", 10);
    auto pubLaserCloudFull_body = node->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_registered_body", 10);
    auto pubLaserCloudEffect = node->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_effected", 10);
    auto pubLaserCloudMap = node->create_publisher<sensor_msgs::msg::PointCloud2>("/Laser_map", 10);
    auto pubPath = node->create_publisher<nav_msgs::msg::Path>("/path", 10);
//------------------------------------------------------------------------------------------------------
    signal(SIGINT,SigHandle);signal(SIGTERM,SigHandle);
    rclcpp::WallRate rate(5000);
    bool status = rclcpp::ok();
    while (status)
    {
        if (flg_exit) break;
        rclcpp::spin_some(node);
        if(fusion_runtime->reset_requested()) {
            lidar_buffer.clear();time_buffer.clear();imu_buffer.clear();integration_imu.clear();
            // Destroy/reconstruct waits for ikd-Tree's rebuild thread before freeing its state.
            ikdtree.~KD_TREE<PointType>();new (&ikdtree) KD_TREE<PointType>();
            state_ikfom initial; kf.change_x(initial);fastlio::Cov P=fastlio::Cov::Identity();kf.change_P(P);
            p_imu=std::make_shared<ImuProcess>();p_imu->set_extrinsic(Lidar_T_wrt_IMU,Lidar_R_wrt_IMU);
            p_imu->set_gyr_cov(V3D::Constant(gyr_cov));p_imu->set_acc_cov(V3D::Constant(acc_cov));p_imu->set_gyr_bias_cov(V3D::Constant(b_gyr_cov));p_imu->set_acc_bias_cov(V3D::Constant(b_acc_cov));p_imu->lidar_type=lidar_type;
            filter_time=-1;last_timestamp_lidar=-1;last_timestamp_imu=-1;lidar_pushed=false;flg_first_scan=true;Localmap_Initialized=false;
            path.poses.clear();feats_undistort->clear();lidar_mean_scantime=0;scan_num=0;state_version=0;
            fusion_runtime->reset(P);
            RCLCPP_WARN(node->get_logger(),"Estimator session reset after time discontinuity or IMU gap");
            continue;
        }
        if(sync_packages(Measures))
        {
            if (flg_first_scan)
            {
                first_lidar_time = Measures.lidar_beg_time;
                p_imu->first_lidar_time = first_lidar_time;
                flg_first_scan = false;
                continue;
            }

            double t0,t1,t2,t3,t4,t5,match_start, solve_start, svd_time;

            match_time = 0;
            kdtree_search_time = 0.0;
            solve_time = 0;
            solve_const_H_time = 0;
            svd_time   = 0;
            t0 = omp_get_wtime();

            feats_undistort->clear();
            if(p_imu->initialized()) {
                if(!fastlio::clipScanToBoundary(*Measures.lidar,Measures.lidar_beg_time,Measures.lidar_end_time,filter_time)){RCLCPP_WARN_THROTTLE(node->get_logger(),*node->get_clock(),1000,"Dropping fully stale LiDAR scan");continue;}
                // Deskew trajectory is isolated from asynchronous auxiliary updates.
                auto deskew_filter=kf;deskew_filter.prediction_observer=nullptr;
                MeasureGroup deskew_measurements=Measures;deskew_measurements.imu.clear();
                for(auto imu:integration_imu){deskew_measurements.imu.push_back(imu);if(stamp_seconds(imu->header.stamp)>=lidar_end_time)break;}
                p_imu->set_state_time(filter_time);
                p_imu->Process(deskew_measurements,deskew_filter,feats_undistort);
                try { fusion_runtime->process(lidar_end_time,predict_until,exported_state,apply_correction,[]{return filter_angular;}); }
                catch(const std::exception& error) {RCLCPP_WARN(node->get_logger(),"%s",error.what());fusion_runtime->request_reset();continue;}
            } else {
                p_imu->Process(Measures,kf,feats_undistort);
                filter_time=lidar_end_time;
                fusion_runtime->filter.reset(kf.get_P());
            }
            fusion_runtime->noise(fastlio::Noise{gyr_cov,acc_cov,b_gyr_cov,b_acc_cov},p_imu->acceleration_scale());
            state_point = kf.get_x();
            pos_lid = state_point.pos + state_point.rot * state_point.offset_T_L_I;

            if (feats_undistort->empty() || (feats_undistort == NULL))
            {
                if(p_imu->initialized())publish_prediction_only(t0);
                continue;
            }

            flg_EKF_inited = (Measures.lidar_beg_time - first_lidar_time) < INIT_TIME ? \
                            false : true;
            /*** Segment the map in lidar FOV ***/
            lasermap_fov_segment();

            /*** downsample the feature points in a scan ***/
            downSizeFilterSurf.setInputCloud(feats_undistort);
            downSizeFilterSurf.filter(*feats_down_body);
            t1 = omp_get_wtime();
            feats_down_size = feats_down_body->points.size();
            /*** initialize the map kdtree ***/
            if(ikdtree.Root_Node == nullptr)
            {
                if(feats_down_size > 5)
                {
                    ikdtree.set_downsample_param(filter_size_map_min);
                    feats_down_world->resize(feats_down_size);
                    for(int i = 0; i < feats_down_size; i++)
                    {
                        pointBodyToWorld(&(feats_down_body->points[i]), &(feats_down_world->points[i]));
                    }
                    ikdtree.Build(feats_down_world->points);
                    ++state_version;fusion_runtime->corrected(exported_state(),nullptr);
                }
                continue;
            }
            int featsFromMapNum = ikdtree.validnum();
            kdtree_size_st = ikdtree.size();

            // cout<<"[ mapping ]: In num: "<<feats_undistort->points.size()<<" downsamp "<<feats_down_size<<" Map num: "<<featsFromMapNum<<"effect num:"<<effct_feat_num<<endl;

            /*** ICP and iterated Kalman filter update ***/
            if (feats_down_size < 5)
            {
                if(p_imu->initialized())publish_prediction_only(t0);
                continue;
            }

            normvec->resize(feats_down_size);
            feats_down_world->resize(feats_down_size);

            V3D ext_euler = SO3ToEuler(state_point.offset_R_L_I);
            fout_pre<<setw(20)<<Measures.lidar_beg_time - first_lidar_time<<" "<<euler_cur.transpose()<<" "<< state_point.pos.transpose()<<" "<<ext_euler.transpose() << " "<<state_point.offset_T_L_I.transpose()<< " " << state_point.vel.transpose() \
            <<" "<<state_point.bg.transpose()<<" "<<state_point.ba.transpose()<<" "<<state_point.grav<< endl;

            if(0) // If you need to see map point, change to "if(1)"
            {
                PointVector ().swap(ikdtree.PCL_Storage);
                ikdtree.flatten(ikdtree.Root_Node, ikdtree.PCL_Storage, NOT_RECORD);
                featsFromMap->clear();
                featsFromMap->points = ikdtree.PCL_Storage;
            }

            pointSearchInd_surf.resize(feats_down_size);
            Nearest_Points.resize(feats_down_size);
            int  rematch_num = 0;
            bool nearest_search_en = true; //

            t2 = omp_get_wtime();

            /*** iterated state estimation ***/
            double t_update_start = omp_get_wtime();
            double solve_H_time = 0;
            const auto prior_state=kf.get_x();const auto prior_cov=kf.get_P();
            kf.update_iterated_dyn_share_modified(LASER_POINT_COV, solve_H_time);
            // Transport current/clone cross covariance through the LIO correction.
            if(fusion_runtime->frontend()) {
                Eigen::Matrix<double,23,1> dx;kf.get_x().boxminus(dx,prior_state);
                fastlio::Cov reset=reset_jacobian(prior_state,dx);
                fusion_runtime->filter.condition(prior_cov,kf.get_P(),dx,reset);
            }
            ++state_version;
            // Analyze once per scan: repeated IEKF iterations must not advance hysteresis.
            Eigen::MatrixXd final_H=Eigen::MatrixXd::Zero(effct_feat_num,6);
            for(int i=0;i<effct_feat_num;++i) {
                const auto& n=corr_normvect->points[i];const auto& p=laserCloudOri->points[i];
                fastlio::Vec3 normal(n.x,n.y,n.z),point(p.x,p.y,p.z);
                fastlio::Vec3 lever=kf.get_x().offset_R_L_I*point+kf.get_x().offset_T_L_I;
                final_H.block<1,3>(i,0)=normal.transpose();
                final_H.block<1,3>(i,3)=(fastlio::skew(lever)*(kf.get_x().rot.conjugate()*normal)).transpose();
            }
            fusion_runtime->degeneracy.analyze(final_H,LASER_POINT_COV,feats_down_size,effct_feat_num?res_mean_last:1.0,true);
            state_point = kf.get_x();
            euler_cur = SO3ToEuler(state_point.rot);
            pos_lid = state_point.pos + state_point.rot * state_point.offset_T_L_I;
            geoQuat.x = state_point.rot.coeffs()[0];
            geoQuat.y = state_point.rot.coeffs()[1];
            geoQuat.z = state_point.rot.coeffs()[2];
            geoQuat.w = state_point.rot.coeffs()[3];

            double t_update_end = omp_get_wtime();

            /******* Publish odometry *******/
            fastlio::Cloud::Ptr key_cloud(new fastlio::Cloud);
            if(!fusion_runtime->degeneracy.status().severe) {
                key_cloud->reserve(feats_down_body->size());
                for(const auto& p:feats_down_body->points){fastlio::Vec3 q=state_point.offset_R_L_I*fastlio::Vec3(p.x,p.y,p.z)+state_point.offset_T_L_I;pcl::PointXYZI point;point.x=q.x();point.y=q.y();point.z=q.z();point.intensity=p.intensity;key_cloud->push_back(point);}
            }
            fusion_runtime->corrected(exported_state(),key_cloud->empty()?fastlio::Cloud::ConstPtr():key_cloud);

            /*** add the feature points to map kdtree ***/
            t3 = omp_get_wtime();
            if(!fusion_runtime->degeneracy.status().severe) map_incremental();
            t5 = omp_get_wtime();
            fusion_runtime->diagnostics(fusion_runtime->degeneracy.status(),lidar_buffer.size(),imu_buffer.size(),1000*(t5-t0));

            /******* Publish points *******/
            if (path_en)                         publish_path(pubPath);
            if (scan_pub_en || pcd_save_en)      publish_frame_world(pubLaserCloudFull);
            if (scan_pub_en && scan_body_pub_en) publish_frame_body(pubLaserCloudFull_body);
            // publish_effect_world(pubLaserCloudEffect);
            // publish_map(pubLaserCloudMap);

            /*** Debug variables ***/
            if (runtime_pos_log)
            {
                frame_num ++;
                kdtree_size_end = ikdtree.size();
                aver_time_consu = aver_time_consu * (frame_num - 1) / frame_num + (t5 - t0) / frame_num;
                aver_time_icp = aver_time_icp * (frame_num - 1)/frame_num + (t_update_end - t_update_start) / frame_num;
                aver_time_match = aver_time_match * (frame_num - 1)/frame_num + (match_time)/frame_num;
                aver_time_incre = aver_time_incre * (frame_num - 1)/frame_num + (kdtree_incremental_time)/frame_num;
                aver_time_solve = aver_time_solve * (frame_num - 1)/frame_num + (solve_time + solve_H_time)/frame_num;
                aver_time_const_H_time = aver_time_const_H_time * (frame_num - 1)/frame_num + solve_time / frame_num;
                T1[time_log_counter] = Measures.lidar_beg_time;
                s_plot[time_log_counter] = t5 - t0;
                s_plot2[time_log_counter] = feats_undistort->points.size();
                s_plot3[time_log_counter] = kdtree_incremental_time;
                s_plot4[time_log_counter] = kdtree_search_time;
                s_plot5[time_log_counter] = kdtree_delete_counter;
                s_plot6[time_log_counter] = kdtree_delete_time;
                s_plot7[time_log_counter] = kdtree_size_st;
                s_plot8[time_log_counter] = kdtree_size_end;
                s_plot9[time_log_counter] = aver_time_consu;
                s_plot10[time_log_counter] = add_point_size;
                time_log_counter ++;
                printf("[ mapping ]: time: IMU + Map + Input Downsample: %0.6f ave match: %0.6f ave solve: %0.6f  ave ICP: %0.6f  map incre: %0.6f ave total: %0.6f icp: %0.6f construct H: %0.6f \n",t1-t0,aver_time_match,aver_time_solve,t3-t1,t5-t3,aver_time_consu,aver_time_icp, aver_time_const_H_time);
                ext_euler = SO3ToEuler(state_point.offset_R_L_I);
                fout_out << setw(20) << Measures.lidar_beg_time - first_lidar_time << " " << euler_cur.transpose() << " " << state_point.pos.transpose()<< " " << ext_euler.transpose() << " "<<state_point.offset_T_L_I.transpose()<<" "<< state_point.vel.transpose() \
                <<" "<<state_point.bg.transpose()<<" "<<state_point.ba.transpose()<<" "<<state_point.grav<<" "<<feats_undistort->points.size()<<endl;
                dump_lio_state_to_log(fp);
            }
        }

        // If LiDAR disappears, keep timestamped auxiliary updates alive with
        // 150 ms latency allowance. Older scans are rejected on resumption.
        if(p_imu->initialized()&&lidar_buffer.empty()&&filter_time>=0&&last_timestamp_imu-filter_time>.3){
            double began=omp_get_wtime();
            try{fusion_runtime->process(last_timestamp_imu-.15,predict_until,exported_state,apply_correction,[]{return filter_angular;});publish_prediction_only(began);}
            catch(const std::exception& error){RCLCPP_WARN(node->get_logger(),"%s",error.what());fusion_runtime->request_reset();}
        }
        status = rclcpp::ok();
        rate.sleep();
    }

    /**************** save map ****************/
    /* 1. make sure you have enough memories
    /* 2. pcd save will largely influence the real-time performences **/
    if (pcl_wait_save->size() > 0 && pcd_save_en)
    {
        string file_name = string("scans.pcd");
        string all_points_dir(string(string(ROOT_DIR) + "PCD/") + file_name);
        pcl::PCDWriter pcd_writer;
        cout << "current scan saved to /PCD/" << file_name<<endl;
        pcd_writer.writeBinary(all_points_dir, *pcl_wait_save);
    }

    fout_out.close();
    fout_pre.close();

    if (runtime_pos_log)
    {
        vector<double> t, s_vec, s_vec2, s_vec3, s_vec4, s_vec5, s_vec6, s_vec7;
        FILE *fp2;
        string log_dir = root_dir + "/Log/fast_lio_time_log.csv";
        fp2 = fopen(log_dir.c_str(),"w");
        fprintf(fp2,"time_stamp, total time, scan point size, incremental time, search time, delete size, delete time, tree size st, tree size end, add point size, preprocess time\n");
        for (int i = 0;i<time_log_counter; i++){
            fprintf(fp2,"%0.8f,%0.8f,%d,%0.8f,%0.8f,%d,%0.8f,%d,%d,%d,%0.8f\n",T1[i],s_plot[i],int(s_plot2[i]),s_plot3[i],s_plot4[i],int(s_plot5[i]),s_plot6[i],int(s_plot7[i]),int(s_plot8[i]), int(s_plot10[i]), s_plot11[i]);
            t.push_back(T1[i]);
            s_vec.push_back(s_plot9[i]);
            s_vec2.push_back(s_plot3[i] + s_plot6[i]);
            s_vec3.push_back(s_plot4[i]);
            s_vec5.push_back(s_plot[i]);
        }
        fclose(fp2);
    }

    RCLCPP_INFO(node->get_logger(),"LiDAR frames received: %d",scan_count);
    fusion_runtime->finish();
    fusion_runtime.reset();
    return 0;
}

int main(int argc,char** argv) {
    int result=0;
    try {result=run_mapping(argc,argv);}
    catch(const std::exception& error){RCLCPP_ERROR(rclcpp::get_logger("fast_lio"),"%s",error.what());result=1;}
    fusion_runtime.reset();node.reset();
    rclcpp::shutdown();return result;
}
