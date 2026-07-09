#!/bin/bash
# 地图质量检测脚本

MAP_FILE="/home/b1/mid360_map/map.pcd"

echo "=========================================="
echo "        地图质量检测工具"
echo "=========================================="
echo ""

if [ ! -f "$MAP_FILE" ]; then
    echo "❌ 错误: 找不到地图文件 $MAP_FILE"
    read -p "按回车退出..."
    exit 1
fi

echo "📁 检测文件: $MAP_FILE"
echo ""

python3 << 'PYEOF'
import numpy as np
import sys

map_file = "/home/b1/mid360_map/map.pcd"

try:
    with open(map_file, 'rb') as f:
        n_points = 0
        fields = ['x','y','z']
        data_type = 'binary'
        sizes = [4]
        while True:
            line = f.readline().decode('ascii', errors='ignore').strip()
            if line.startswith('FIELDS'):
                fields = line.split()[1:]
            if line.startswith('SIZE'):
                sizes = [int(s) for s in line.split()[1:]]
            if line.startswith('POINTS'):
                n_points = int(line.split()[1])
            if line.startswith('DATA'):
                data_type = line.split()[1]
                break
        
        point_size = sum(sizes)
        if data_type == 'ascii':
            raw = f.read()
            data = np.loadtxt(raw.decode('utf-8').strip().split('\n'), dtype=np.float32)
            if data.ndim == 1:
                data = data.reshape(-1, len(fields))
        else:
            raw = f.read(n_points * point_size)
            data = np.frombuffer(raw, dtype=np.float32).reshape(n_points, -1)
        
        points = data
        
        x, y, z = points[:, 0], points[:, 1], points[:, 2]
        
        print("=" * 50)
        print("📊 基本信息")
        print("=" * 50)
        print(f"  总点数: {n_points}")
        print(f"  X范围: {x.min():.2f} ~ {x.max():.2f} (跨度 {x.max()-x.min():.2f}m)")
        print(f"  Y范围: {y.min():.2f} ~ {y.max():.2f} (跨度 {y.max()-y.min():.2f}m)")
        print(f"  Z范围: {z.min():.2f} ~ {z.max():.2f} (跨度 {z.max()-z.min():.2f}m)")
        print(f"  中心: ({x.mean():.2f}, {y.mean():.2f}, {z.mean():.2f})")
        
        # 密度分析
        volume = (x.max()-x.min()) * (y.max()-y.min()) * (z.max()-z.min())
        density = n_points / volume if volume > 0 else 0
        
        print("")
        print("=" * 50)
        print("📈 密度分析")
        print("=" * 50)
        print(f"  体积: {volume:.0f} m³")
        print(f"  平均密度: {density:.2f} 点/m³")
        
        # 区域分布
        x_mid = (x.min() + x.max()) / 2
        y_mid = (y.min() + y.max()) / 2
        
        q1 = int(np.sum((x < x_mid) & (y < y_mid)))
        q2 = int(np.sum((x >= x_mid) & (y < y_mid)))
        q3 = int(np.sum((x < x_mid) & (y >= y_mid)))
        q4 = int(np.sum((x >= x_mid) & (y >= y_mid)))
        
        print("")
        print("=" * 50)
        print("🗺️ 区域分布 (按中心分4个象限)")
        print("=" * 50)
        print(f"  左下({x.min():.0f}~{x_mid:.0f}, {y.min():.0f}~{y_mid:.0f}): {q1} 点")
        print(f"  右下({x_mid:.0f}~{x.max():.0f}, {y.min():.0f}~{y_mid:.0f}): {q2} 点")
        print(f"  左上({x.min():.0f}~{x_mid:.0f}, {y_mid:.0f}~{y.max():.0f}): {q3} 点")
        print(f"  右上({x_mid:.0f}~{x.max():.0f}, {y_mid:.0f}~{y.max():.0f}): {q4} 点")
        
        # 网格空洞检查
        print("")
        print("=" * 50)
        print("🔍 空洞检查 (5m网格)")
        print("=" * 50)
        
        grid_size = 5.0
        x_bins = np.arange(x.min(), x.max(), grid_size)
        y_bins = np.arange(y.min(), y.max(), grid_size)
        
        empty_cells = 0
        sparse_cells = 0
        total_cells = 0
        for xi in x_bins:
            for yi in y_bins:
                count = int(np.sum((x >= xi) & (x < xi+grid_size) & (y >= yi) & (y < yi+grid_size)))
                total_cells += 1
                if count == 0:
                    empty_cells += 1
                elif count < 10:
                    sparse_cells += 1
        
        print(f"  网格总数: {total_cells}")
        print(f"  空网格(0点): {empty_cells} ({empty_cells/total_cells*100:.1f}%)")
        print(f"  稀疏网格(<10点): {sparse_cells} ({sparse_cells/total_cells*100:.1f}%)")
        print(f"  正常网格: {total_cells - empty_cells - sparse_cells} ({(total_cells - empty_cells - sparse_cells)/total_cells*100:.1f}%)")
        
        # 综合评分
        print("")
        print("=" * 50)
        print("⭐ 综合评分")
        print("=" * 50)
        
        score = 0
        issues = []
        
        # 点数评分 (30分)
        if n_points >= 500000:
            score += 30
            print(f"  ✅ 点数充足 ({n_points})  +30分")
        elif n_points >= 100000:
            score += 20
            print(f"  ⚠️ 点数一般 ({n_points})  +20分")
            issues.append("点数偏少，建议多走动建图")
        else:
            score += 10
            print(f"  ❌ 点数不足 ({n_points})  +10分")
            issues.append("点数太少，需要重新建图")
        
        # 密度评分 (20分)
        if density >= 5:
            score += 20
            print(f"  ✅ 密度良好 ({density:.2f} 点/m³)  +20分")
        elif density >= 2:
            score += 15
            print(f"  ⚠️ 密度一般 ({density:.2f} 点/m³)  +15分")
            issues.append("密度偏低，建议减小体素滤波")
        else:
            score += 5
            print(f"  ❌ 密度不足 ({density:.2f} 点/m³)  +5分")
            issues.append("密度太低，需要重新建图")
        
        # 覆盖面积评分 (25分)
        area = (x.max()-x.min()) * (y.max()-y.min())
        if area >= 1000:
            score += 25
            print(f"  ✅ 覆盖面积大 ({area:.0f} m²)  +25分")
        elif area >= 400:
            score += 20
            print(f"  ⚠️ 覆盖面积一般 ({area:.0f} m²)  +20分")
            issues.append("覆盖面积偏小")
        else:
            score += 10
            print(f"  ❌ 覆盖面积小 ({area:.0f} m²)  +10分")
            issues.append("覆盖面积太小，需要多走动")
        
        # 均匀性评分 (25分)
        uniformity = (total_cells - empty_cells) / total_cells * 100
        if uniformity >= 70:
            score += 25
            print(f"  ✅ 分布均匀 (覆盖{uniformity:.1f}%)  +25分")
        elif uniformity >= 40:
            score += 15
            print(f"  ⚠️ 分布一般 (覆盖{uniformity:.1f}%)  +15分")
            issues.append("点云分布不均，有大片空洞")
        else:
            score += 5
            print(f"  ❌ 分布不均 (覆盖{uniformity:.1f}%)  +5分")
            issues.append("点云分布严重不均，需要重新建图")
        
        # 最终结果
        print("")
        print("=" * 50)
        if score >= 80:
            print(f"🎉 总分: {score}/100 - 地图质量优秀！")
            print("   可以直接用于重定位")
        elif score >= 60:
            print(f"⚠️ 总分: {score}/100 - 地图质量一般")
            print("   可以尝试重定位，但可能不太稳定")
        else:
            print(f"❌ 总分: {score}/100 - 地图质量不合格")
            print("   建议重新建图")
        
        if issues:
            print("")
            print("📋 改进建议:")
            for i, issue in enumerate(issues, 1):
                print(f"   {i}. {issue}")
        
        print("=" * 50)
        
except Exception as e:
    print(f"❌ 读取地图文件失败: {e}")
    sys.exit(1)
PYEOF

echo ""
read -p "按回车退出..."
