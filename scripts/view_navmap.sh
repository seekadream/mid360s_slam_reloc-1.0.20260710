#!/bin/bash
# 导航地图查看器 - 最终版
echo "=========================================="
echo "  导航地图查看器"
echo "=========================================="

MAP_YAML="$HOME/mid360_map/navmap.yaml"

if [ ! -f "$MAP_YAML" ]; then
    echo "错误: 地图文件不存在: $MAP_YAML"
    echo ""
    echo "请先运行 '生成导航地图' 创建地图"
    read -p "按回车键关闭..."
    exit 1
fi

echo "地图文件: $MAP_YAML"
echo ""

source /opt/ros/humble/setup.bash
source "$HOME/mid360_slam_ws/install/setup.bash"
export ROS_DOMAIN_ID=30

# 清理旧进程
pkill -9 -f "map_server" 2>/dev/null
pkill -9 -f "rviz2" 2>/dev/null
sleep 1

trap 'kill 0; exit 0' SIGINT SIGTERM

gnome-terminal --title="导航地图查看器" -- bash -c "
source /opt/ros/humble/setup.bash
source /home/pi/mid360_slam_ws/install/setup.bash
export ROS_DOMAIN_ID=30

echo '启动 map_server...'
ros2 run nav2_map_server map_server --ros-args -p yaml_filename:=\$HOME/mid360_map/navmap.yaml -p topic_name:=map -p frame_id:=camera_init &
MAP_PID=\$!
sleep 5

echo '配置 map_server...'
ros2 lifecycle set /map_server configure 2>&1 | grep -v '^\[INFO\]'
sleep 3

echo '激活 map_server...'
ros2 lifecycle set /map_server activate 2>&1 | grep -v '^\[INFO\]'
sleep 3

echo ''
echo '启动 rviz2...'
rviz2 -d \$HOME/mid360_slam_ws/src/mid360_slam/config/navmap.rviz &
RVIZ_PID=\$!

echo ''
echo '=========================================='
echo '导航地图已加载'
echo '如果看不到地图，请在rviz2中：'
echo '  1. 确保Map已勾选'
echo '  2. 确认Topic是/map'
echo '  3. 尝试滚轮缩放'
echo '=========================================='
echo '按回车关闭...'
read
kill \$MAP_PID \$RVIZ_PID 2>/dev/null
"

echo "导航地图查看器已启动"
echo "请在新打开的终端窗口中查看"