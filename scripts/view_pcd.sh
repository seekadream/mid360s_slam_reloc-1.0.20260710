#!/bin/bash
# PCD 点云文件查看器 - 改进版
PCD_FILE="${1:-$HOME/mid360_map/map.pcd}"

echo "=========================================="
echo "  PCD 点云地图查看器"
echo "=========================================="

if [ ! -f "$PCD_FILE" ]; then
    echo "错误: 文件不存在: $PCD_FILE"
    echo ""
    echo "可用的PCD文件:"
    ls -lh "$HOME/mid360_map"/*.pcd 2>/dev/null || echo "  (无)"
    echo ""
    echo "请先运行 MID360一体化 建图"
    read -p "按回车键关闭..."
    exit 1
fi

echo "PCD文件: $PCD_FILE"
echo "文件大小: $(du -h "$PCD_FILE" | awk '{print $1}')"
echo ""

ROS2_SETUP="/opt/ros/humble/setup.bash"
source "$ROS2_SETUP"
source "$HOME/mid360_slam_ws/install/setup.bash"

# 清理旧进程
pkill -9 -f "pcd_to_pointcloud" 2>/dev/null
pkill -9 -f "rviz2" 2>/dev/null
sleep 1

trap 'kill 0; exit 0' INT TERM

echo "启动PCD发布器..."
ros2 run pcl_ros pcd_to_pointcloud --ros-args -p file_name:="$PCD_FILE" -p frame_id:="camera_init" -p topic:="/cloud_pcd" -r __name:=pcd_publisher &
PUB_PID=$!
sleep 2

echo "启动Rviz2..."
# 创建临时rviz配置
TMP_RVIZ="/tmp/pcd_viewer.rviz"
cat > "$TMP_RVIZ" << 'EOF'
Panels:
  - Class: rviz_common/Displays
    Name: Displays
  - Class: rviz_common/Views
    Name: Views
Visualization Manager:
  Class: ""
  Displays:
    - Class: rviz_default_plugins/PointCloud2
      Name: PointCloud
      Topic: /cloud_pcd
      Color Transformer: AxisColor
      Axis: Z
      Max Color: 255; 0; 0
      Min Color: 0; 0; 255
      Max Value: 2
      Min Value: -1
      Size (Pixels): 3
      Style: Points
      Alpha: 1
      Decay Time: 0
      Value: true
    - Class: rviz_default_plugins/Grid
      Name: Grid
      Cell Size: 1
      Color: 160; 160; 160
      Value: true
    - Class: rviz_default_plugins/TF
      Name: TF
      Show Names: false
      Value: true
  Global Options:
    Background Color: 20; 20; 30
    Fixed Frame: camera_init
    Frame Rate: 30
  Name: root
  Tools:
    - Class: rviz_default_plugins/MoveCamera
    - Class: rviz_default_plugins/FocusCamera
  Value: true
  Views:
    Current:
      Class: rviz_default_plugins/Orbit
      Distance: 20
      Focal Point:
        X: 0
        Y: 0
        Z: 0
      Pitch: 0.5
      Yaw: 3.14159
      Value: Orbit
Window Geometry:
  Height: 900
  Width: 1400
  X: 0
  Y: 0
EOF

rviz2 -d "$TMP_RVIZ" &
RVIZ_PID=$!

echo ""
echo "=========================================="
echo "PCD地图已加载: $PCD_FILE"
echo ""
echo "如果看不到点云:"
echo "  1. 检查左侧面板 PointCloud 是否勾选"
echo "  2. 尝试用鼠标滚轮缩放"
echo "  3. 按住左键旋转视角"
echo "=========================================="
echo "按 Ctrl+C 关闭"

wait $PUB_PID $RVIZ_PID