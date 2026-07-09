# MID360 SLAM Tools

Livox MID-360 激光雷达的一键建图 + NDT 重定位工具包。

---

## 📁 目录结构

```
mid360_all_in_one/
├── *.desktop                 # 桌面快捷方式文件
├── start.sh                  # MID360 SLAM 建图一键启动
├── run_slam.sh               # SLAM 建图 + 自动保存地图
├── start_mid360_slam.sh      # MID360 SLAM 启动脚本
├── start_mapping_20cm.sh     # 建图（过滤20cm内近点）
├── start_relocalization.sh   # NDT 重定位一键启动（交互版）
├── start_reloc_headless.sh   # NDT 重定位启动（无交互，开机自启用）
├── pose_monitor.py           # 实时位姿监控（输出 X,Y,Z,Yaw,Roll,Pitch + 精度）
├── rviz_reloc.rviz           # 重定位用 RViz2 配置文件
├── lidar-network.sh           # 开机自启网络路由脚本
├── set-display-resolution.sh  # 开机自启分辨率脚本
│
├── config/                   # 配置文件
│   ├── mid360.yaml            # FAST_LIO 建图参数（默认）
│   ├── mid360_filter20cm.yaml # FAST_LIO 建图参数（20cm过滤版）
│   ├── mid360_legged.yaml     # NDT 重定位参数
│   ├── mid360s_localization.rviz  # 重定位 RViz2 配置
│   ├── MID360s_config.json    # Livox 雷达驱动网络配置
│   ├── MID360_config.json     # Livox MID360 配置
│   ├── HAP_config.json        # Livox HAP 配置
│   └── mixed_HAP_MID360_config.json # 混合配置
│
├── scripts/                  # 工具脚本
│   ├── generate_navmap.py     # PCD → 2D 导航地图生成
│   ├── check_map_quality.sh   # 检测 PCD 地图质量评分
│   ├── print_pose.py          # 实时输出位姿到终端
│   ├── view_pcd.py            # 查看 PCD 点云文件
│   ├── view_pcd.sh            # 查看 PCD 快捷脚本
│   ├── view_navmap.sh         # 查看导航地图
│   ├── gen_navmap.sh          # 导航地图生成脚本
│   ├── capture_pointcloud.py  # 采集点云数据
│   ├── capture_auto.py        # 自动采集
│   ├── relocalization.py      # 重定位辅助脚本
│   ├── save_coordinates.py    # 保存坐标到文件
│   ├── coordinate_monitor.py  # 坐标监控
│   ├── coordinate_display.py  # 坐标显示
│   ├── mouse_teleop.py        # 鼠标遥控
│   ├── check_imu.py           # IMU 诊断
│   └── record_imu.py          # IMU 数据记录
│
├── src/                      # 源码
│   ├── FAST_LIO/              # FAST-LIO SLAM 算法源码
│   ├── livox_ros_driver2/     # Livox 雷达 ROS2 驱动
│   └── mid360_slam/           # MID360SLAM 建图启动包
│
└── lidarloc_project/          # NDT 重定位项目
    ├── start_relocalization.sh   # 重定位一键启动
    ├── start_reloc_headless.sh   # 重定位无交互启动
    ├── pose_monitor.py           # 位姿监控
    ├── rviz_reloc.rviz           # 重定位 RViz2 配置
    └── lidarloc_ws/src/
        ├── lidar_localization_ros2/  # NDT 重定位核心算法
        └── ndt_omp_ros2/             # NDT 多线程加速库
```

---

## 🚀 快捷文件说明

| 桌面图标 | 功能 | 对应命令 |
|---|---|---|
| `MID360SLAM建图.desktop` | 一键启动 SLAM 建图（默认参数） | `bash start.sh` |
| `MID360SLAM建图_20cm过滤.desktop` | 过滤 20cm 内近点的建图版本 | `bash start_mapping_20cm.sh` |
| `MID360S重定位启动.desktop` | 在已建地图中定位（NDT） | `bash start_relocalization.sh` |
| `检测地图质量.desktop` | 检测 PCD 地图是否适合重定位 | `bash scripts/check_map_quality.sh` |

---

## 🔧 配置文件说明

### FAST_LIO 建图参数 (`config/mid360.yaml`)

| 参数 | 默认值 | 说明 |
|---|---|---|
| `preprocess/blind` | `4` | 过滤雷达最近距离（米），4=过滤4米内 |
| `mapping/acc_cov` | `0.1` | IMU加速度噪声协方差 |
| `mapping/gyr_cov` | `0.1` | IMU陀螺仪噪声协方差 |
| `preprocess/lidar_type` | `1` | 雷达类型（1=Livox） |
| `point_filter_num` | `2` | 点云抽稀（每隔N个点取1个） |

### NDT 重定位参数 (`config/mid360_legged.yaml`)

| 参数 | 默认值 | 说明 |
|---|---|---|
| `ndt_resolution` | `1.5` | NDT 网格大小（米），越小越精细 |
| `ndt_step_size` | `0.2` | NDT 步长 |
| `score_threshold` | `6.0` | 匹配分数阈值，超限则拒绝 |
| `registration_method` | `NDT_OMP` | 匹配方法（NDT_OMP / GICP） |

### Livox 雷达网络配置 (`config/MID360s_config.json`)

```json
{
  "host_ip": "192.168.0.108",       // 电脑 IP
  "lidar_configs": [{
    "ip": "192.168.0.126",           // 雷达 IP
    "pcl_data_type": 1               // 点云格式
  }]
}
```

---

## 📜 启动脚本说明

### `start.sh` — MID360 SLAM 建图

启动流程：
1. 设置 ROS_DOMAIN_ID
2. 启动 livox 雷达驱动
3. 启动 FAST_LIO SLAM 建图节点
4. 启动 RViz2 可视化
5. 启动位姿输出脚本
6. 每 30 秒自动保存 PCD 地图到 `~/mid360_map/`

### `start_relocalization.sh` — NDT 重定位

启动流程：
1. 清理旧 ROS 进程
2. 发布 TF（`odom→base_link`）
3. 启动 Livox 雷达驱动（PointCloud2 格式）
4. 启动 NDT 重定位节点（加载已有 PCD 地图）
5. 启动 RViz2 可视化
6. 启动位姿监控

---

## 🖥️ 系统配置

| 文件 | 说明 |
|---|---|
| `lidar-network.sh` | 开机自启：配置雷达专用网络路由 |
| `set-display-resolution.sh` | 开机自启：设置屏幕分辨率 1920×1080 |

### 网络架构

```
Windows/MobaXterm ←→ WiFi (192.168.0.100) ←→ Ubuntu
                                              ↕ (eth0, 192.168.0.108)
                                           LiDAR (192.168.0.126)
```

- WiFi 用于远程连接（SSH/RDP）
- eth0 直连雷达
- 策略路由确保两条路径互不干扰

---

## 📊 脚本工具

| 脚本 | 功能 |
|---|---|
| `scripts/generate_navmap.py` | 将 3D PCD 点云转换为 2D 导航地图（PGM+YAML） |
| `scripts/check_map_quality.sh` | 检测 PCD 地图质量并打分（0-100） |
| `scripts/view_pcd.py` | 快速查看 PCD 点云文件 |
| `scripts/view_navmap.sh` | 查看生成的导航地图 |
| `scripts/capture_pointcloud.py` | 采集并保存当前帧点云数据 |
| `scripts/print_pose.py` | 实时输出当前位姿 |
| `scripts/check_imu.py` | IMU 传感器诊断 |
| `scripts/record_imu.py` | 录制 IMU 数据 |

---

## 📝 输出文件

| 文件 | 说明 |
|---|---|
| `~/mid360_map/map.pcd` | 建图生成的 PCD 点云地图 |
| `~/Desktop/pose_log.txt` | 定位日志（时间、坐标、精度、姿态） |
| `/tmp/mid360_reloc/` | 重定位运行时日志 |

---

## 📋 依赖

- ROS2 Humble
- PCL (Point Cloud Library)
- Eigen3
- Sophus
- OpenMP
