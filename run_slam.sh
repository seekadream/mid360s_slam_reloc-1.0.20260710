#!/bin/bash
# ============================================================
#  MID360S SLAM 一键启动 + 自动保存地图
#  功能: LiDAR驱动 -> SLAM定位建图 -> RViz可视化
#        -> 定时保存 PCD 点云地图 + 导航地图 (pgm+yaml)
# ============================================================
set -e

WORKSPACE="$HOME/mid360_slam_ws"
INSTALL_SETUP="$WORKSPACE/install/setup.bash"
ROS2_SETUP="/opt/ros/humble/setup.bash"
MAP_DIR="$HOME/mid360_map"
NAVSCRIPT="$WORKSPACE/scripts/generate_navmap.py"
PCD_DIR="$WORKSPACE/build/fast_lio"

red='\033[0;31m'; green='\033[0;32m'; yellow='\033[1;33m'; blue='\033[0;34m'; nc='\033[0m'
print_info()  { echo -e "${blue}[信息]${nc} $1"; }
print_ok()    { echo -e "${green}[成功]${nc} $1"; }
print_warn()  { echo -e "${yellow}[警告]${nc} $1"; }
print_error() { echo -e "${red}[错误]${nc} $1"; }

cleanup() {
    echo ""
    print_info "正在停止所有 SLAM 进程..."
    pkill -2 -f "fastlio_mapping" 2>/dev/null
    pkill -2 -f "livox_ros_driver2" 2>/dev/null
    pkill -2 -f "rviz2" 2>/dev/null
    pkill -2 -f "print_pose.py" 2>/dev/null
    sleep 2
    print_ok "所有进程已停止"
    exit 0
}
trap cleanup SIGINT SIGTERM

mkdir -p "$MAP_DIR"

if [ ! -f "$INSTALL_SETUP" ]; then
    print_error "工作空间未编译, 请先编译"
    read -p "按回车键退出..."
    exit 1
fi

source "$ROS2_SETUP"
source "$INSTALL_SETUP"

export ROS_DOMAIN_ID=30

echo ""
echo -e "${green}╔══════════════════════════════════════════════╗${nc}"
echo -e "${green}║    MID360S LiDAR SLAM 定位建图系统        ║${nc}"
echo -e "${green}║    Livox MID-360S + FAST-LIO + ROS2 Humble ║${nc}"
echo -e "${green}║   PCD点云 + 导航地图自动保存 + 实时位姿   ║${nc}"
echo -e "${green}╚══════════════════════════════════════════════╝${nc}"
echo ""

print_info "启动 SLAM 系统..."
ros2 launch mid360_slam mid360_slam.launch.py use_rviz:=true &
SLAM_PID=$!
sleep 6

print_info "启动自动保存 (PCD + 导航地图, 每30秒)..."
while true; do
    sleep 30
    TS=$(date +%Y%m%d_%H%M%S)

    # 1. 保存 PCD 点云地图
    ros2 service call /map_save std_srvs/srv/Trigger "{}" 2>/dev/null
    sleep 2

     SRC=""
    [ -f "$PCD_DIR/map.pcd" ] && SRC="$PCD_DIR/map.pcd"
    [ -f "$WORKSPACE/map.pcd" ] && SRC="$WORKSPACE/map.pcd"
    [ -f "./map.pcd" ] && SRC="./map.pcd"
    [ -f "$MAP_DIR/map.pcd" ] && SRC="$MAP_DIR/map.pcd"
    [ -f "$HOME/mid360_map/map.pcd" ] && SRC="$HOME/mid360_map/map.pcd"

    if [ -n "$SRC" ] && [ -f "$SRC" ]; then
        PCD_FILE="$MAP_DIR/map_${TS}.pcd"
        cp "$SRC" "$PCD_FILE"
        PCD_SIZE=$(ls -lh "$PCD_FILE" | awk '{print $5}')
        print_ok "PCD: map_${TS}.pcd [${PCD_SIZE}]"

        # 2. 生成导航地图
        if [ -f "$NAVSCRIPT" ]; then
            NAV_NAME="navmap_${TS}"
            python3 "$NAVSCRIPT" "$PCD_FILE" \
                -o "$MAP_DIR" \
                -n "$NAV_NAME" \
                -r 0.05 \
                --zmin -0.8 --zmax 2.0 \
                --min-points 2 \
                2>&1 | tail -1
        fi
    else
        print_warn "未找到 PCD 文件, 跳过"
    fi
done &
SAVE_PID=$!

print_ok "系统已启动 (SLAM PID=$SLAM_PID | 自动保存 PID=$SAVE_PID)"
print_info "地图目录: $MAP_DIR"
print_info " PCD 点云: map_*.pcd"
print_info " 导航地图: navmap_*.pgm + navmap_*.yaml"
print_info " 实时位姿: TIME,X,Y,Z,YAW,VX,VY,VZ,|V| (通过 /Odometry)"
print_info "按 Ctrl+C 停止所有进程"
echo ""

wait $SLAM_PID
