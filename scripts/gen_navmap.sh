#!/bin/bash
# 导航地图生成器 (从 PCD 生成 pgm+yaml) - 改进版
NAME="${2:-navmap}"

if [ -n "$1" ] && [ -f "$1" ]; then
    PCD="$1"
else
    echo "查找 $HOME/mid360_map 下最新的 PCD 文件..."
    # 优先使用map.pcd（最新的完整地图），然后才是带时间戳的文件
    if [ -f "$HOME/mid360_map/map.pcd" ]; then
        PCD="$HOME/mid360_map/map.pcd"
        echo "使用完整地图: $PCD"
    else
        LATEST=$(ls -t "$HOME/mid360_map"/map_*.pcd 2>/dev/null | head -1)
        if [ -n "$LATEST" ]; then
            echo "使用最新文件: $LATEST"
            PCD="$LATEST"
        else
            echo "错误: $HOME/mid360_map 下未找到任何 PCD 文件"
            echo "请先启动 MID360一体化 建图并等待自动保存"
            exit 1
        fi
    fi
fi

echo "输入: $PCD"
echo "文件大小: $(du -h "$PCD" | awk '{print $1}')"
echo "点数: $(python3 -c "
with open('$PCD','rb') as f:
    while True:
        l=f.readline().decode().strip()
        if l.startswith('POINTS'): print(l.split()[1]); break
" 2>/dev/null || echo '未知')"

echo ""
echo "开始生成导航地图..."
echo "参数: 分辨率=0.05m, 高度=[-0.1, 2.0]m, 最小占据点=3, 膨胀=2像素"
echo ""

python3 "$HOME/mid360_slam_ws/scripts/generate_navmap.py" \
    "$PCD" \
    -o "$HOME/mid360_map" \
    -n "$NAME" \
    -r 0.05 \
    --zmin -0.1 --zmax 2.0 \
    --min-points 3 \
    --inflate 2

echo ""
echo "地图文件:"
ls -lh "$HOME/mid360_map/${NAME}.pgm" "$HOME/mid360_map/${NAME}.yaml" 2>/dev/null

echo ""
echo "查看地图: 点击桌面上的 '查看导航地图' 快捷方式"