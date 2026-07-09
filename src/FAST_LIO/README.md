# FAST-LIO2 NDT 地图重定位 (ROS 2 Humble)

基于 [FAST-LIO2](https://github.com/hku-mars/FAST_LIO) 和 [S-FAST-LIO](https://github.com/zlwang7/S-FAST_LIO)，适配 MID-360S 雷达的 NDT 地图重定位。

原始仓库: [fast-lio2-map-based-localization](https://github.com/xz00/fast-lio2-map-based-localization)

## 原理

1. 加载预构建的全局 PCD 地图
2. 在 RVIZ2 中用 "2D Pose Estimate" 给出粗略初始位姿
3. 使用 NDT 将当前帧点云与局部地图匹配，完成精确定位
4. 结合 IMU 前向传播 + 点云匹配，通过贝叶斯更新实时跟踪位姿

## 硬件要求

- MID-360S 雷达（或 Velodyne/Ouster 等）
- 雷达 IP: `192.168.0.126`，主机 IP: `192.168.0.5`
- Ubuntu 22.04 + ROS 2 Humble

## 依赖

```bash
sudo apt install ros-humble-pcl-ros ros-humble-tf2-ros ros-humble-rviz2
# Eigen3, PCL, Sophus, OpenMP 通常已默认安装
```

## 编译

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/seekadream/fast-lio2-relocalization.git sfast_lio
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select sfast_lio
source install/setup.bash
```

## 准备工作

### 1. 构建全局地图

推荐使用 [FAST-LIO-SAM](https://github.com/kahowang/FAST_LIO_SAM) 或 [FAST-LIO2](https://github.com/hku-mars/FAST_LIO) 建图，生成 PCD 格式的全局地图。

### 2. 修改配置文件

编辑 `config/velodyne_re.yaml`:

```yaml
# 全局地图路径
globalmap_dir: "/home/pi/mid360_map/map.pcd"

# 雷达 topic
lid_topic: "/livox/lidar"

# IMU topic
imu_topic: "/livox/imu"
```

关键参数:

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `init_threshold` | `0.15` | NDT 匹配分数低于此值才算初始化成功 |
| `init_resolution_rot` | `1` | 初始化旋转搜索分辨率（度） |
| `init_resolution_dis` | `0.05` | 初始化平移搜索分辨率（米） |
| `max_iteration` | `3` | NDT 最大迭代次数 |
| `filter_size_surf` | `0.5` | 点云降采样尺寸 |

## 运行

### 启动 MID-360S 雷达驱动

```bash
source ~/mid360_slam_ws/install/setup.bash
ros2 launch livox_ros_driver2 msg_MID360s_pcl2_launch.py
```

### 启动重定位

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch sfast_lio mapping_velodyne_relocalization.launch.py
```

### 初始化

1. 在 RVIZ2 工具栏点击 **"2D Pose Estimate"**
2. 在地图上点击你的大致位置
3. 拖动箭头指向朝向
4. NDT 会自动收敛到精确位姿

### 键盘精确调整初始化

如果粗略初始化失败，可以手动微调：

```bash
source install/setup.bash
ros2 run sfast_lio keyboard_catch_local.py
```

| 按键 | 功能 |
|:--:|:--:|
| `W/S` | 前/后移动 |
| `A/D` | 左/右移动 |
| `Q/E` | 左/右旋转 |
| `J/L` | 减小/增大旋转分辨率 |
| `I/K` | 减小/增大位移分辨率 |
| `F` | 完成调整，开始 NDT 匹配 |

## 输出 Topic

| Topic | 类型 | 说明 |
|-------|------|------|
| `/Odometry` | `nav_msgs/Odometry` | IMU + 点云匹配融合结果 |
| `/Odometry_relocal` | `nav_msgs/Odometry` | 仅点云与局部地图匹配结果 |
| `/path` | `nav_msgs/Path` | 位姿轨迹 |
| `/registered_cloud` | `sensor_msgs/PointCloud2` | 匹配后的点云 |

## 致谢

- [FAST-LIO2](https://github.com/hku-mars/FAST_LIO)
- [S-FAST-LIO](https://github.com/zlwang7/S-FAST_LIO)
- [FAST-LIO-SAM](https://github.com/kahowang/FAST_LIO_SAM)
- [LIO-SAM relocalization](https://github.com/Gaochao-hit/LIO-SAM_based_relocalization)
