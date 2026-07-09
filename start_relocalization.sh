#!/bin/bash
# MID-360S 重定位一键启动 (NDT重定位)
# 使用已建好的 PCD 地图进行重定位

ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-30}"
MAP_FILE="/home/b1/mid360_map/map.pcd"
EXPECTED_LIDAR_IP="192.168.0.126"
LOG_DIR="/tmp/mid360_reloc"
mkdir -p "$LOG_DIR"

export ROS_DOMAIN_ID

# 清理旧进程
for p in static_transform_publisher fastlio_mapping livox_ros_driver2_node lidar_localization rviz2; do
    ps aux | grep "$p" | grep -v grep | awk '{print $2}' | xargs -r kill -9 2>/dev/null
done
sleep 3

source /opt/ros/humble/setup.bash
source /home/b1/mid360_slam_ws/install/setup.bash
source /home/b1/Desktop/lidarloc_project/lidarloc_ws/install/setup.bash || {
    echo "❌ lidarloc_ws 编译状态异常"; read -p "按回车退出..."; exit 1
}

[ -f "$MAP_FILE" ] || { echo "❌ 未找到地图"; read -p "按回车退出..."; exit 1; }

echo "地图: $MAP_FILE"
ping -c 1 -W 1 $EXPECTED_LIDAR_IP >/dev/null 2>&1 && echo "雷达 ✅" || echo "雷达 ⚠️"

cleanup() {
    echo ""; echo ">>> 停止..."; kill $ODOM_TF_PID $LIVOX_PID $LOC_PID $RVIZ_PID $POSE_PID 2>/dev/null; wait 2>/dev/null; exit 0
}
trap cleanup EXIT INT TERM

# TF
echo "[0/3] odom->base_link..."
ros2 run tf2_ros static_transform_publisher --x 0 --y 0 --z 0 --roll 0 --pitch 0 --yaw 0 --frame-id odom --child-frame-id base_link > "$LOG_DIR/tf.log" 2>&1 &
ODOM_TF_PID=$!
sleep 2

# 雷达驱动
echo "[1/3] 雷达驱动..."
ros2 run livox_ros_driver2 livox_ros_driver2_node \
    --ros-args -p xfer_format:=0 -p data_src:=0 \
    -p publish_freq:=10.0 -p output_data_type:=0 \
    -p frame_id:=livox_frame \
    -p user_config_path:="/home/b1/Desktop/mid360_slam_ws/install/livox_ros_driver2/share/livox_ros_driver2/config/MID360s_config.json" \
    > "$LOG_DIR/livox.log" 2>&1 &
LIVOX_PID=$!

for i in $(seq 1 30); do
    ros2 topic list 2>/dev/null | grep -q "/livox/lidar" && break
    kill -0 $LIVOX_PID 2>/dev/null || { echo "❌ 雷达驱动退出"; exit 1; }
    sleep 1
done
echo ">>> 雷达就绪 ✅"

# NDT 重定位
echo "[2/3] NDT 重定位..."
ros2 launch lidar_localization_ros2 mid360_legged_localization.launch.py \
    map_path:="$MAP_FILE" \
    cloud_topic:=/livox/lidar \
    imu_topic:=/livox/imu \
    set_initial_pose:=true \
    initial_pose_z:=1.0 \
    use_imu_preintegration:=false \
    use_continuous_time_deskew:=false \
    > "$LOG_DIR/ndt.log" 2>&1 &
LOC_PID=$!
sleep 5
kill -0 $LOC_PID 2>/dev/null || { echo "❌ 重定位启动失败"; exit 1; }
echo ">>> 重定位已启动 ✅"

# 3. RViz2
echo "[3/3] 启动 RViz2..."

RVIZ_CFG="/home/b1/Desktop/lidarloc_project/lidar_localization_ros2/rviz/mid360s_localization.rviz"
[ -f "$RVIZ_CFG" ] && rviz2 -d "$RVIZ_CFG" > "$LOG_DIR/rviz.log" 2>&1 &
RVIZ_PID=$!
sleep 3

echo ">>> 等待地图加载和定位就绪..."
for i in $(seq 1 20); do
    if ! kill -0 $LOC_PID 2>/dev/null; then
        cat "$LOG_DIR/ndt.log" | grep -E "Map Size|Activating|fitness" | tail -3
        echo "❌ 重定位已退出"; read -p "按回车退出..."; exit 1
    fi
    if grep -q "Activating end" "$LOG_DIR/ndt.log" 2>/dev/null; then
        echo ">>> 定位已就绪 ✅ (${i}s)"; break
    fi
    sleep 1
done

# 位姿监控
echo "[4/4] 启动位姿监控..."
python3 /home/b1/Desktop/lidarloc_project/pose_monitor.py &
POSE_PID=$!

echo ""
echo "✅ 重定位系统已就绪"
echo "日志目录: $LOG_DIR"

while kill -0 $LOC_PID 2>/dev/null; do sleep 1; done

echo ">>> 重定位已退出"
read -p "按回车关闭..."
exit 0
