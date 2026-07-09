#!/bin/bash
# MID360 SLAM 建图 (过滤20cm以内近点)

INSTALL_CONFIG="/home/b1/Desktop/mid360_slam_ws/install/fast_lio/share/fast_lio/config/mid360.yaml"
SRC_CONFIG="/home/b1/Desktop/mid360_slam_ws/src/FAST_LIO/config/mid360_filter20cm.yaml"
BAK_CONFIG="/tmp/mid360_config.bak"

# 退出时恢复原配置
trap 'cp "$BAK_CONFIG" "$INSTALL_CONFIG" 2>/dev/null; rm -f "$BAK_CONFIG"' EXIT

# 备份原配置，替换为20cm过滤配置
cp "$INSTALL_CONFIG" "$BAK_CONFIG" 2>/dev/null
cp "$SRC_CONFIG" "$INSTALL_CONFIG"

# 运行建图
cd /home/b1/mid360_slam_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
bash start.sh
