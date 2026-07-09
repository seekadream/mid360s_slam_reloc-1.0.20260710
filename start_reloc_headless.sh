#!/bin/bash
# MID-360S 重定位一键启动 (无交互模式, 用于开机自启)

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-30}"
MAP_FILE="/home/b1/mid360_map/map.pcd"
LOG_DIR="/tmp/mid360_reloc"
mkdir -p "$LOG_DIR"

# 清理旧进程
for p in static_transform_publisher fastlio_mapping livox_ros_driver2_node lidar_localization rviz2; do
    ps aux | grep "$p" | grep -v grep | awk '{print $2}' | xargs -r kill -9 2>/dev/null
done
sleep 3

source /opt/ros/humble/setup.bash
source /home/b1/mid360_slam_ws/install/setup.bash
source /home/b1/Desktop/lidarloc_project/lidarloc_ws/install/setup.bash || exit 1
[ -f "$MAP_FILE" ] || exit 1

# TF
ros2 run tf2_ros static_transform_publisher --x 0 --y 0 --z 0 --roll 0 --pitch 0 --yaw 0 \
    --frame-id odom --child-frame-id base_link > "$LOG_DIR/tf.log" 2>&1 &
ODOM_TF_PID=$!
sleep 2

# 雷达驱动
ros2 run livox_ros_driver2 livox_ros_driver2_node \
    --ros-args -p xfer_format:=0 -p data_src:=0 \
    -p publish_freq:=10.0 -p output_data_type:=0 \
    -p frame_id:=livox_frame \
    -p user_config_path:="/home/b1/Desktop/mid360_slam_ws/install/livox_ros_driver2/share/livox_ros_driver2/config/MID360s_config.json" \
    -r /livox/lidar:=/reloc/lidar -r /livox/imu:=/reloc/imu \
    > "$LOG_DIR/livox.log" 2>&1 &
LIVOX_PID=$!

for i in $(seq 1 30); do
    ros2 topic list 2>/dev/null | grep -q "/reloc/lidar" && break
    kill -0 $LIVOX_PID 2>/dev/null || exit 1
    sleep 1
done

# NDT 重定位
ros2 launch lidar_localization_ros2 mid360_legged_localization.launch.py \
    map_path:="$MAP_FILE" cloud_topic:=/reloc/lidar imu_topic:=/reloc/imu \
    use_continuous_time_deskew:=false set_initial_pose:=true \
    use_imu_preintegration:=false enable_map_odom_tf:=true \
    registration_method:=NDT_OMP \
    > "$LOG_DIR/ndt.log" 2>&1 &
LOC_PID=$!
sleep 5
kill -0 $LOC_PID 2>/dev/null || exit 1

# RViz2
RVIZ_CFG="/home/b1/Desktop/lidarloc_project/rviz_reloc.rviz"
[ -f "$RVIZ_CFG" ] && rviz2 -d "$RVIZ_CFG" > "$LOG_DIR/rviz.log" 2>&1 &
RVIZ_PID=$!

# 位姿监控
python3 /home/b1/Desktop/lidarloc_project/pose_monitor.py &
POSE_PID=$!

# 保持运行
while kill -0 $LOC_PID 2>/dev/null; do sleep 1; done
