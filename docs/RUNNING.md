# ROS2 Humble 多传感器 SLAM

本分支把 FAST-LIO2 迁移到 ROS2 Humble。默认仅启用激光、IMU、退化处理和回环后端。辅助传感器必须提供真实标定后再启用。

## 构建与资源限制

依赖 Ubuntu 22.04、ROS2 Humble、Eigen、PCL、OpenCV、Boost、CMake；ROS 包包括 `pcl_conversions`、`cv_bridge`、`tf2_ros`、`ament_cmake_gtest`。GTSAM 使用官方 4.2 源码，压缩包 SHA256 固定在脚本中；Livox 驱动为可选项。

已安装 ROS2 Humble 并配置官方软件源后，补充依赖：

```bash
sudo apt install build-essential cmake git curl libeigen3-dev libpcl-dev libopencv-dev \
  libboost-all-dev python3-colcon-common-extensions ros-humble-pcl-conversions \
  ros-humble-cv-bridge ros-humble-tf2-ros ros-humble-ament-cmake-gtest \
  ros-humble-diagnostic-msgs ros-humble-rosidl-default-generators \
  ros-humble-rosbag2-storage-default-plugins
```

```bash
source /opt/ros/humble/setup.bash
./scripts/bootstrap.sh
./scripts/build.sh
source install/setup.bash
colcon test --packages-select fast_lio --executor sequential
colcon test-result --verbose
```

`bootstrap.sh` 和 `build.sh` 均经过 `with_limits.sh`：单任务、nice 15、空闲磁盘优先级；本机有 user systemd 时，CPU 限为半个核心、内存最多 3 GB、禁止构建使用 swap。内存不足时构建会失败，而不是无限使用内存。没有 user systemd 时降级为单核心、单任务和低优先级。可通过 `FASTLIO_CPU_QUOTA`、`FASTLIO_MEMORY_HIGH`、`FASTLIO_MEMORY_MAX` 显式调整。

可选 Livox：先安装 `livox_ros_driver2`，再执行 `./scripts/build.sh -DFASTLIO_LIVOX=ON`。这个可选分支尚未在本机验证；标准 PointCloud2 分支已编译运行。

运行时点云匹配默认使用 2 个工作线程，与编译单任务限制分别控制。可用 `./scripts/build.sh -DFASTLIO_MAPPING_THREADS=1` 减少运行时占用（范围 1–4）。

## 启动

```bash
ros2 launch fast_lio slam.launch.py
# 数据回放：必须把下面文件换成对应数据集的完整标定和 topic 配置
ros2 launch fast_lio slam.launch.py config:=/absolute/path/rig.yaml use_sim_time:=true
ros2 bag play /absolute/path/normalized_bag --clock 100 --rate 1.0
```

原有传感器配置已改为 ROS2 参数格式，提供 `mapping_avia.launch.py`、`mapping_mid360.launch.py` 等启动文件。仓库保留的 `.launch` 文件属于上游 ROS1 历史文件，不安装到 ROS2 包。

已下载数据的示例：

```bash
ros2 launch fast_lio slam.launch.py config:="$PWD/config/fusionportable_v2.yaml" use_sim_time:=true
# 另一个已 source ROS 环境的终端
ros2 bag play data/fusionportable_v2/normalized_60s --clock 100 --rate 1.0
```

这个配置启用轮速和双目后端融合，GPS 因缺少官方天线杆臂标定而关闭。它适用于已转换的 UGV 数据，不能直接用于其他机器人。

需要叠加多个配置时可以直接运行：

```bash
ros2 run fast_lio fastlio_mapping --ros-args \
  --params-file config/slam.yaml \
  --params-file /absolute/path/rig-calibration.yaml \
  --params-file config/multisensor.yaml \
  -p fusion.mode:=frontend -p use_sim_time:=true
```

`calibration.template.yaml` 中的单位外参只是格式示例。`imu_T_sensor` 表示 **把 sensor 坐标转换到 IMU 坐标**；四元数顺序为 x,y,z,w。激光外参继续使用 `mapping.extrinsic_T/R`。`imu_T_camera` 是原始左相机光学系外参，程序根据 CameraInfo.R 转换到校正后的光学系。

## 输入与融合选择

| 输入 | 消息 | 默认 topic | 要求 |
|---|---|---|---|
| 激光 | PointCloud2 | `/points_raw` | 每点采样时间；雷达类型和时间单位必须匹配 |
| IMU | Imu | `/imu/data` | 加速度含重力，角速度 rad/s；初始阶段需静止 |
| 轮速 | TwistWithCovarianceStamped | `/wheel/twist` | `frame_id=base_link`；linear.x 为前向 m/s；covariance[0]>0 |
| GPS | NavSatFix | `/gps/fix` | 有效状态、有限经纬高、正定已知位置协方差 |
| 左右图像 | Image | `/camera/{left,right}/image_raw` | 原始双目；mono8 或可由 cv_bridge 转换的编码 |
| 相机标定 | CameraInfo | `/camera/{left,right}/camera_info` | 正确 K/D/R/P，左 P 的 Tx=0，正基线及共同垂直投影 |

`fusion.mode` 仅允许 `frontend` 或 `backend`，运行期间修改会被拒绝。三类辅助传感器统一选边；独立的 `wheel/gnss/stereo.enabled` 控制是否接入。配置参数按启动配置使用，不支持传感器或模型的在线重新配置。

- 前端模式：轮速、GPS、双目更新局部估计；后端不会添加这些原始观测因子。
- 后端模式：局部估计只使用激光和 IMU，辅助因子只进入后端，因此 GPS 等不会直接修正 `/odometry/imu`。
- 后端不开启时，不能同时选择 backend 模式并启用辅助传感器。
- `wheel.nonholonomic` 默认开启，限轮式平台；手持数据应关闭轮速及这个约束。
- GPS 航向未知时，等待 GNSS 和本地轨迹均有至少 `gnss.alignment_baseline` 米位移后对齐；静止时不会强行估计航向。已知航向时配置 `gnss.yaw_known=true` 和 `gnss.enu_to_odom_yaw`（rad）。

IMU 修正时间为原始时间减 `common.time_offset_lidar_to_imu`；辅助传感器为原始时间加各自 `time_offset`。不启用旧版 `common.time_sync_en` 自动对时。

硬件默认使用传感器 best-effort QoS。大点云本地 bag 回放建议 `common.reliable_inputs=true`，避免 DDS 丢包；发布端也必须支持 reliable。本数据集配置已开启。输入坐标变换使用配置外参，不根据输入消息的任意 frame_id 自动查 TF。

局部地图要求 `cube_side_length > 3 × mapping.det_range`，不满足时启动报错，防止窗口移动后误删局部地图。默认边长 1000 m；AVIA 预设在 450 m 检测范围下使用 1800 m。ROS1 启动文件中的滤波尺寸、迭代次数等已迁移到对应 YAML。双目前端克隆数越多计算量越大，UGV 示例使用 6 个历史克隆。

缺少 CameraInfo 的图像 bag 可用标定文件发布相同时间戳的 CameraInfo：

```bash
ros2 run fast_lio publish_camera_info.py --left left.yaml --right right.yaml
```

文件采用 ROS `camera_calibration` 的 `image_width/height`、`camera_matrix`、`distortion_coefficients`、`rectification_matrix`、`projection_matrix` 格式。不能把未完成双目校正的两个独立内参文件当成有效 stereo 标定。

## 输出与故障行为

| 输出 | 坐标系 / 内容 |
|---|---|
| `/odometry/local` | odom 中的 base_link；估计器校正或降级预测结果 |
| `/odometry/imu` | 每个有效 IMU 样本一次，odom 中的 base_link；频率受输入 IMU 限制 |
| `/odometry/global` | map 中的 base_link；后端更新频率 |
| `/path/optimized` | 后端优化后的 base_link 路径 |
| `/map/global` | 按关键帧优化位姿生成的全局点云 |
| `/diagnostics` | 退化方向数、匹配比例、队列、辅助观测计数和后端因子数量 |
| TF | map→odom→base_link；base_link 到 IMU、激光、左相机、GPS 的静态外参 |

高频递推使用独立线程。新的局部校正状态到来时重放历史 IMU，重放过程不重复发布旧时间戳。后端只改变 map→odom，不把状态反馈到局部前端。

IMU 缺测超过默认 100 ms 后，不继续发布无效递推，诊断会报警。IMU 时间倒退或不可积分的 IMU 间隔触发新会话初始化。激光消失时，系统以 150 ms 延迟余量继续处理辅助观测；恢复时，早于已提交状态的点云被丢弃。严重激光匹配失败暂停地图插入，恢复需通过滞回检查。辅助观测迟到、溢出和异常值均有计数。

相邻扫描仅部分时间重叠时，裁掉已处理点并重设扫描起点，保留剩余点的绝对采样时刻。非有限 IMU 样本拒绝并计数，初始化时拒绝零加速度。

## 导出

```bash
ros2 service call /fastlio_mapping/save std_srvs/srv/Trigger '{}'
```

也会在正常 SIGINT/SIGTERM 退出时导出。位置由 `output.directory` 指定，默认 `/tmp/fastlio-output`：`trajectory.tum` 是优化后的 base_link 位姿，`map.pcd` 是全局地图，`keyframes.csv` 和 `keyframe_<node>.pcd` 保存关键帧索引与 IMU 系点云。未实现跨会话合图和地图加载重定位。

## 验证

```bash
# 节点级合成测试，按顺序执行；不要与编译同时运行
ROS_DOMAIN_ID=87 ROS_LOCALHOST_ONLY=1 python3 scripts/ros_smoke.py --mode backend
ROS_DOMAIN_ID=88 ROS_LOCALHOST_ONLY=1 python3 scripts/ros_smoke.py --mode frontend
ROS_DOMAIN_ID=89 ROS_LOCALHOST_ONLY=1 python3 scripts/ros_smoke.py --mode frontend --faults
```

测试检查模式不可动态切换、互斥融合、200 Hz 消息输出、延迟、TF、协方差和地图导出。合成测试不作为真实数据精度结论。真实数据访问状态和工具见 [DATASETS.md](DATASETS.md)，已完成与未完成的验收见 [VALIDATION.md](VALIDATION.md)。
