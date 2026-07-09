#!/bin/bash
# ============================================================
#  MID360S 一键启动 (SLAM窗口 + 位姿窗口)
# ============================================================
WORKSPACE="$HOME/mid360_slam_ws"
SETUP="$WORKSPACE/install/setup.bash"
ROS2="/opt/ros/humble/setup.bash"

cleanup() {
    pkill -2 -f "fastlio_mapping" 2>/dev/null || true
    pkill -2 -f "livox_ros_driver2" 2>/dev/null || true
    pkill -2 -f "rviz2" 2>/dev/null || true
    pkill -2 -f "print_pose.py" 2>/dev/null || true
    exit 0
}
trap cleanup SIGINT SIGTERM

# ===== 窗口1: SLAM + 地图保存 =====
gnome-terminal --title="MID360 SLAM 建图" -- bash -c "
export ROS_DOMAIN_ID=30
source $ROS2
source $SETUP
trap 'exit 0' SIGINT SIGTERM

echo -e '\033[0;32m╔════════════════════════════════════════╗\033[0m'
echo -e '\033[0;32m║   MID360S SLAM 定位建图              ║\033[0m'
echo -e '\033[0;32m╚════════════════════════════════════════╝\033[0m'
echo ''
echo -e '\033[0;33m地图保存位置: ~/mid360_map/\033[0m'
echo ''

ros2 launch mid360_slam mid360_slam.launch.py use_rviz:=true use_pose_printer:=false &
SLAM_PID=\$!
sleep 8

mkdir -p ~/mid360_map
while true; do
    sleep 30
    TS=\$(date +%Y%m%d_%H%M%S)
    ros2 service call /map_save std_srvs/srv/Trigger '{}' 2>/dev/null || true
    sleep 2
    PCD_FOUND=''
    for d in ~/mid360_slam_ws/build/fast_lio ~/mid360_slam_ws; do
        if [ -f \"\$d/test.pcd\" ]; then
            PCD_FOUND=\"\$d/test.pcd\"
            break
        fi
    done
    if [ -z \"\$PCD_FOUND\" ]; then
        PCD_FOUND=\$(find ~/mid360_slam_ws -name '*.pcd' -newer ~/mid360_slam_ws/install/setup.bash 2>/dev/null | head -1)
    fi
    if [ -n \"\$PCD_FOUND\" ]; then
        cp \"\$PCD_FOUND\" ~/mid360_map/map_\${TS}.pcd 2>/dev/null
        cp \"\$PCD_FOUND\" ~/mid360_map/map.pcd 2>/dev/null
        echo -e '\033[0;32m[保存]\033[0m map_\${TS}.pcd'
        echo -e '\033[0;32m[更新]\033[0m map.pcd (最新完整地图)'
        du -h ~/mid360_map/map.pcd | awk '{print \"  文件大小: \" \$1}'
    else
        echo -e '\033[0;31m[错误]\033[0m 未找到 PCD 文件'
    fi
done &
SAVE_PID=\$!
wait \$SLAM_PID
kill \$SAVE_PID 2>/dev/null
echo ''
echo 'SLAM 已停止'
echo -e '\033[0;33m地图保存在: ~/mid360_map/\033[0m'
echo '按回车关闭窗口...'
read
"

# ===== 窗口2: 实时位姿 =====
sleep 2
gnome-terminal --title="实时位姿" -- bash -c "
export ROS_DOMAIN_ID=30
source $ROS2
source $SETUP
trap 'exit 0' SIGINT SIGTERM
python3 $WORKSPACE/scripts/print_pose.py
echo ''
echo '位姿输出已停止'
echo -e '\033[0;33m位姿日志保存在: ~/桌面/pose_log_*.txt\033[0m'
echo '按回车关闭窗口...'
read
"