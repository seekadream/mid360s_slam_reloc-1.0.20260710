#!/bin/bash
# ============================================================
#  MID360S LiDAR SLAM 一键启动脚本
#  功能: 启动 LiDAR 驱动 -> SLAM 定位建图 -> RViz 可视化
#
#  用法: ./start_mid360_slam.sh [选项]
#    -n    无 RViz (仅建图)
#    -s    仅保存地图 (需SLAM已在运行)
#    -h    显示帮助
# ============================================================

set -e

WORKSPACE="$HOME/mid360_slam_ws"
INSTALL_SETUP="$WORKSPACE/install/setup.bash"
ROS2_SETUP="/opt/ros/humble/setup.bash"
MAP_DIR="$HOME/mid360_map"

# 颜色输出
red='\033[0;31m'
green='\033[0;32m'
yellow='\033[1;33m'
blue='\033[0;34m'
nc='\033[0m'

print_info()  { echo -e "${blue}[信息]${nc} $1"; }
print_ok()    { echo -e "${green}[成功]${nc} $1"; }
print_warn()  { echo -e "${yellow}[警告]${nc} $1"; }
print_error() { echo -e "${red}[错误]${nc} $1"; }

show_banner() {
    echo ""
    echo -e "${green}╔══════════════════════════════════════════════╗${nc}"
    echo -e "${green}║    MID360S LiDAR SLAM 定位建图系统        ║${nc}"
    echo -e "${green}║    Livox MID-360S + FAST-LIO + ROS2 Humble ║${nc}"
    echo -e "${green}╚══════════════════════════════════════════════╝${nc}"
    echo ""
}

check_network() {
    print_info "检查网络连接..."
    if ip addr show | grep -q "192.168.1"; then
        print_ok "LiDAR 网络接口已配置"
    else
        print_warn "未检测到 192.168.1.x 网段, 请确保已连接 LiDAR 并配置 IP"
        print_warn "  主机 IP: 192.168.1.5 (需配为静态)"
        print_warn "  LiDAR IP: 192.168.1.12"
    fi
}

save_map() {
    local MAP_PATH="${MAP_DIR}/map_$(date +%Y%m%d_%H%M%S).pcd"
    print_info "正在保存地图到: $MAP_PATH"
    mkdir -p "$MAP_DIR"

    source "$ROS2_SETUP"
    source "$INSTALL_SETUP"

    ros2 service call /save_map std_srvs/srv/Trigger "{}" 2>/dev/null || {
        print_warn "SLAM 服务调用失败, 尝试直接复制当前 PCD..."
        if [ -f "$WORKSPACE/build/fast_lio/test.pcd" ]; then
            cp "$WORKSPACE/build/fast_lio/test.pcd" "$MAP_PATH"
            print_ok "地图已保存: $MAP_PATH"
        elif [ -f "./test.pcd" ]; then
            cp "./test.pcd" "$MAP_PATH"
            print_ok "地图已保存: $MAP_PATH"
        else
            print_error "未找到地图文件 (test.pcd)"
            return 1
        fi
    }
    print_ok "地图保存完成: $MAP_PATH"
}

start_slam() {
    local USE_RVIZ="${1:-true}"

    show_banner

    # 检查配置文件
    if [ ! -f "$INSTALL_SETUP" ]; then
        print_error "工作空间未编译, 请先运行: cd $WORKSPACE && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release"
        exit 1
    fi

    check_network

    # 创建地图目录
    mkdir -p "$MAP_DIR"

    print_info "激活 ROS2 环境..."
    source "$ROS2_SETUP"
    source "$INSTALL_SETUP"

    print_info "启动 MID360S SLAM 系统..."
    echo ""
    print_warn "================================================"
    print_warn "  请确认 LiDAR 广播码已正确配置!"
    print_warn "  请在 mid360_slam.launch.py 中修改 CMD_LINE_BD_CODE"
    print_warn "  当前广播码: livox0000000001"
    print_warn "================================================"
    echo ""

    if [ "$USE_RVIZ" = "false" ]; then
        print_info "启动模式: 仅建图 (无 RViz)"
        ros2 launch mid360_slam mid360_slam.launch.py use_rviz:=false
    else
        print_info "启动模式: 建图 + RViz 可视化"
        ros2 launch mid360_slam mid360_slam.launch.py use_rviz:=true
    fi
}

# 主入口
case "${1:-}" in
    -s|--save)
        save_map
        ;;
    -n|--no-rviz)
        start_slam false
        ;;
    -h|--help)
        echo "用法: $0 [选项]"
        echo ""
        echo "选项:"
        echo "  无参数    完整启动 (LiDAR驱动 + SLAM + RViz)"
        echo "  -n        仅建图, 不启动 RViz"
        echo "  -s        保存当前地图 (需 SLAM 已在运行)"
        echo "  -h        显示此帮助"
        echo ""
        echo "配置文件:"
        echo "  LiDAR配置: src/livox_ros_driver2/config/MID360s_config.json"
        echo "  SLAM配置:  src/FAST_LIO/config/mid360.yaml"
        echo "  启动配置:  src/mid360_slam/launch/mid360_slam.launch.py"
        echo ""
        echo "使用流程:"
        echo "  1. 修改广播码: CMD_LINE_BD_CODE"
        echo "  2. 连接 LiDAR, 配置网络 (主机IP: 192.168.1.5)"
        echo "  3. 运行: $0       (启动 SLAM)"
        echo "  4. 保存: $0 -s    (保存地图)"
        ;;
    *)
        start_slam true
        ;;
esac
