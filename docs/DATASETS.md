# 数据集准备与真实回放

## 官方来源与访问状态

核查日期：2026-10-29。

| 数据集 | 选择目的 | 本次访问结果 |
|---|---|---|
| [FusionPortableV2](https://fusionportable.github.io/dataset/fusionportable_v2/) | UGV 上的双目、激光、IMU、轮编码器及 GNSS 接入 | `ugv_parking00` 已下载、解压并完成 topic/标定检查；10 月 28 日配额限制后，29 日重试成功 |
| [ENWIDE](https://projects.asl.ethz.ch/datasets/enwide/) | 隧道等几何退化；全站仪位置真值 | 获取了官方说明和 DOI 元数据；[官方 Research Collection](https://doi.org/10.3929/ethz-c-000787551) 返回 HTTP 429，尚未取得 bag |

原始压缩包 4,063,287,652 字节，解压 bag 15,622,861,399 字节；7z CRC 检查通过。本地 SHA256 为 `350240b3d99007204a594c3674b1d56a9bf778843cd144588b2067941304cc98`，官方未提供对照 hash，因此不宣称官方哈希校验通过。来源记录见 [dataset-provenance.json](validation-results/dataset-provenance.json)。数据与派生 bag 合计约 28.52 GB（十进制），未进入版本控制。

FusionPortableV2 官方特别说明点云 header 是 **扫描结束时刻**，每点 `t` 为纳秒。FAST-LIO 的输入应转换成扫描开始时刻和相对开始的逐点时间。不可直接套用起始时间配置。

实际检查：179.008 s，1790 帧 Ouster 点云，35801 条 STIM300 IMU（200 Hz），3581 对双目，16189 条编码器，左右各 358 条原始 GNSS NavSatFix。GNSS 有有效状态和位置协方差，但官方标定未给出天线杆臂，因此真实实验关闭 GNSS，不使用融合 INS 输出替代。完整 topic 清单保留在 `data/fusionportable_v2/topics.json`。

生成的 `normalized_60s` 包含 600 帧激光、11999 条 IMU、1201 对图像和 CameraInfo、5380 条轮速；`normalized_full_lidar_wheel` 保留全程激光/IMU/轮速/原始 GNSS，省略图像以控制磁盘使用，运行时 GNSS 仍关闭。rosbag2 SQLite/metadata v8 已用 Humble 工具实际读取。

## Python 环境

数据工具与 ROS 运行环境分开安装，避免替换 ROS 的 numpy/cv_bridge 依赖：

```bash
python3 -m venv .deps/data-env
.deps/data-env/bin/pip install -r scripts/requirements.txt
```

若系统没有 python3-venv，也可安装到独立目录：

```bash
python3 -m pip install --target .deps/python-tools -r scripts/requirements.txt
PYTHONPATH="$PWD/.deps/python-tools" python3 tests/test_tools.py
```

这里的 PYTHONPATH 只用于离线工具，不应写入全局 shell 配置。

## 下载、检查、转换

```bash
.deps/data-env/bin/python scripts/dataset.py download \
  --source fusionportable_v2 --name ugv_parking00_rosbag.7z \
  --output data/fusionportable_v2
```

下载工具检查文件大小及 30 GB 预算。清单有官方 SHA256 时执行对照；没有时只记录本地 hash，不把本地计算结果称为官方校验。Google Drive 配额限制需要等待官方恢复；工具不会绕过访问限制。解压前检查剩余空间，并保留官方标定和参考轨迹。

```bash
.deps/data-env/bin/python scripts/dataset.py inspect data/sequence.bag > topics.json
# 用 inspect 得到的真实 topic 替换下列示例。只转换显式列出的来源。
.deps/data-env/bin/python scripts/dataset.py convert data/sequence.bag data/normalized \
  --topic /SOURCE_POINTS:=/points_raw --topic /SOURCE_IMU:=/imu/data \
  --topic /SOURCE_LEFT:=/camera/left/image_raw \
  --topic /SOURCE_RIGHT:=/camera/right/image_raw \
  --point-time-field t --point-time-unit ns --lidar-header end \
  --decompress-images --seconds 120
```

转换后的统一点云字段为 `x,y,z,intensity,time,ring`，逐点 `time` 单位为秒；配置 `preprocess.lidar_type=2`、`preprocess.timestamp_unit=0`。转换保留原始消息采集时间，不根据方位角伪造时间。没有逐点时间或时间含义不明时拒绝转换。

编码器 CSV 的规范输入为 `timestamp,left_ticks,right_ticks`：

```bash
.deps/data-env/bin/python scripts/dataset.py wheel ticks.csv wheel.csv \
  --radius YOUR_RADIUS_METERS --ticks-per-revolution YOUR_TICKS \
  --variance 0.04 --counter-modulus 65536
```

`--counter-modulus` 只在原始计数器确实有对应回绕范围时使用。`wheel.csv` 为 `timestamp,speed,variance`，可传入转换命令的 `--wheel-csv wheel.csv`，生成 `/wheel/twist`。轮径、计数定义及时间单位必须来自对应序列的传感器说明。

外参导入工具接受已确认方向的四个 4×4 变换 `imu_T_base/imu_T_camera/imu_T_gnss/imu_T_lidar`，输出 ROS2 参数：

```bash
.deps/data-env/bin/python scripts/dataset.py calibration transforms.yaml rig-calibration.yaml
```

工具校验刚体变换，不能自动推断官方文件中未声明的坐标方向。相机 K/D/R/P 使用对应序列官方双目标定和 `publish_camera_info.py`。

## 本次 UGV 数据的转换配方

使用 [官方 20230426 标定](https://github.com/fusionportable/calibration_files/tree/master/20230426_calib) 和 [编码器转换定义](https://github.com/fusionportable/fusionportable_dataset_tools/tree/main/calibration/encoder_calc)。下载标定目录内的 `body_imu`、`ouster00`、`ouster00_imu`、`frame_cam00`、`frame_cam01`、`3dm_imu`、`mini_hercules_wheel` 对应 YAML 到 `data/fusionportable_v2/calibration`。实际文件及来源树已保留在本机数据目录。

```bash
export PYTHONPATH="$PWD/.deps/python-tools"
python3 scripts/fusionportable.py calibrate data/fusionportable_v2/calibration data/fusionportable_v2/imported
python3 scripts/fusionportable.py wheel data/fusionportable_v2/raw/ugv_parking00/ugv_parking00.bag \
  --output data/fusionportable_v2/wheel.csv
python3 scripts/dataset.py convert \
  data/fusionportable_v2/raw/ugv_parking00/ugv_parking00.bag data/fusionportable_v2/normalized_60s \
  --topic /os_cloud_node/points:=/points_raw \
  --topic /stim300/imu/data_raw:=/imu/data \
  --topic /stereo/frame_left/image_raw/compressed:=/camera/left/image_raw \
  --topic /stereo/frame_right/image_raw/compressed:=/camera/right/image_raw \
  --point-time-field t --point-time-unit ns --lidar-header end --decompress-images \
  --wheel-csv data/fusionportable_v2/wheel.csv \
  --camera-info /camera/left/image_raw:=data/fusionportable_v2/imported/left.yaml \
  --camera-info /camera/right/image_raw:=data/fusionportable_v2/imported/right.yaml --seconds 60
```

目标目录必须不存在；不要覆盖现有派生数据。脚本按官方轮径 0.217 m、计数到轮周角的定义和回绕处理，输出区间中点时间的速度。16186 个区间有效，2 个区间被拒绝。相机用原始双目相对外参重新生成 R/P，校正后基线约 0.159828 m；分别应用左右相机固定时间偏移。激光标定显式求逆得到 IMU_T_lidar。生成配置固定外参，不在线估计。

## 消融与评测

```bash
python3 scripts/benchmark.py data/normalized --config /absolute/path/complete-rig.yaml
.deps/data-env/bin/python scripts/evaluate.py results/ablations/all_backend/odometry reference.tum \
  --bag-topic /odometry/global --reference-source '说明真值设备及是否与输入 GNSS 同源' \
  --output metrics.json
```

消融包含纯 LIO、退化处理、各辅助传感器单独前端融合、全部前端、全部后端以及关闭回环。每项都启动独立进程，记录 odometry、诊断、峰值 RSS 和回放状态。评测使用时间关联和 SE(3) 对齐，不拟合尺度；缺少时间重叠或有效轨迹时明确失败。

ENWIDE 的全站仪参考是位置真值；没有姿态真值时不能把人工补上的单位四元数作为真实姿态评估旋转 RPE。位置 ATE 可单独评估，旋转指标需标记不可用。

本次官方停车轨迹位于 `data/fusionportable_v2/ugv_parking00_reference.tum`，5422 个位姿。其姿态约定与车体运动方向存在约 90° 差异，参考点和独立性也未完全核实。因此仅进行**假定参考位置属于 body_imu 的条件性位置比较**：将估计的 base_link 按已知杆臂转换到 IMU，再做 SE(3) 对齐；不使用参考四元数转换位置、不评估旋转 RPE、不拟合尺度。该结果不能替代参考坐标约定确认后的正式精度验收。

```bash
PYTHONPATH="$PWD/.deps/python-tools" python3 scripts/evaluate.py \
  results/fusionportable_60s/all_backend/odometry data/fusionportable_v2/ugv_parking00_reference.tum \
  --bag-topic /odometry/global --position-only \
  --estimate-to-imu data/fusionportable_v2/imported/transforms.yaml \
  --reference-source 'Official ugv_parking00; body_imu reference-point assumption; orientation and independence unverified; GNSS disabled' \
  --output metrics.json
```

实际运行结果与局限见 [VALIDATION.md](VALIDATION.md)。

本轮消融的复现入口（新输出目录，顺序运行，不与编译并行）：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ROS_DOMAIN_ID=92 ROS_LOCALHOST_ONLY=1 nice -n 15 ionice -c 3 python3 scripts/benchmark.py \
  data/fusionportable_v2/normalized_60s --config config/fusionportable_v2.yaml \
  --output results/reproduction_60s
# 全程派生 bag 没有图像，显式限制为轮速；不能声称该组验证了全程双目融合
ROS_DOMAIN_ID=92 ROS_LOCALHOST_ONLY=1 nice -n 15 ionice -c 3 python3 scripts/benchmark.py \
  data/fusionportable_v2/normalized_full_lidar_wheel --config config/fusionportable_v2.yaml \
  --output results/reproduction_full --sensors wheel --variant all_backend --variant all_backend_no_loop
PYTHONPATH="$PWD/.deps/python-tools" python3 scripts/collect_validation.py \
  results/reproduction_60s results/reproduction_full \
  --reference data/fusionportable_v2/ugv_parking00_reference.tum \
  --transforms data/fusionportable_v2/imported/transforms.yaml --output results/reproduction.json
```

汇总脚本要求各运行已完成，计算在线输出及最终优化轨迹的条件性位置指标，并保存实际观测计数、模式、队列与代码哈希。前后端的比较使用各自的 local/global 输出，后端模式额外保留 local 对照。默认消融中 GNSS 单传感器组因缺少可用标定自动跳过。
