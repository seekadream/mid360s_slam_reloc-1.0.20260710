#!/usr/bin/env python3
"""
MID360 SLAM 导航地图生成器 (改进版)
功能: 将 3D PCD 点云地图 转换为 ROS2 Nav2 兼容的 2D 占据栅格地图 (*.pgm + *.yaml)

改进:
  1. 更好的高度过滤
  2. 点云降采样
  3. 更准确的占据判断
  4. 膨胀处理优化
"""
import sys
import os
import struct
import numpy as np
import argparse
from datetime import datetime

# ═══════════════════════════════════════════════════════════
#  PCD 读取
# ═══════════════════════════════════════════════════════════

def read_pcd(filepath):
    """读取 PCD 文件, 返回 (x, y, z, intensity) 数组"""
    with open(filepath, 'rb') as f:
        header = {}
        while True:
            line = f.readline().decode('utf-8').strip()
            if not line:
                continue
            key, _, value = line.partition(' ')
            header[key] = value
            if key == 'DATA':
                break

        if not header.get('VERSION', '').startswith(('.7', '0.')):
            raise ValueError(f"不支持的 PCD 版本: {header.get('VERSION')}")

        fields = header['FIELDS'].split()
        types = header['TYPE'].split()
        sizes = [int(s) for s in header['SIZE'].split()]
        width = int(header['WIDTH'])
        height = int(header['HEIGHT'])
        points = int(header.get('POINTS', width * height))
        data_type = header['DATA']
        point_size = sum(sizes)

        raw = f.read()

        if data_type == 'ascii':
            data = np.loadtxt(raw.decode('utf-8').strip().split('\n'), dtype=np.float32)
            if data.ndim == 1:
                data = data.reshape(-1, len(fields))
        elif data_type == 'binary':
            data = np.frombuffer(raw[:points * point_size], dtype=np.float32).reshape(points, -1)
        else:
            raise ValueError(f"不支持的数据类型: {data_type}")

        out = {}
        for i, name in enumerate(fields):
            if i < data.shape[1]:
                out[name] = data[:, i]
        return out, points


# ═══════════════════════════════════════════════════════════
#  点云降采样
# ═══════════════════════════════════════════════════════════

def voxel_downsample(x, y, z, voxel_size=0.05):
    """体素降采样"""
    if len(x) == 0:
        return x, y, z
    
    # 计算体素索引
    x_min, y_min, z_min = x.min(), y.min(), z.min()
    xi = np.floor((x - x_min) / voxel_size).astype(np.int32)
    yi = np.floor((y - y_min) / voxel_size).astype(np.int32)
    zi = np.floor((z - z_min) / voxel_size).astype(np.int32)
    
    # 使用字典存储每个体素的点
    voxel_dict = {}
    for i in range(len(x)):
        key = (xi[i], yi[i], zi[i])
        if key not in voxel_dict:
            voxel_dict[key] = []
        voxel_dict[key].append(i)
    
    # 对每个体素取中心点
    indices = []
    for idx_list in voxel_dict.values():
        indices.append(idx_list[0])  # 取第一个点
    
    return x[indices], y[indices], z[indices]


# ═══════════════════════════════════════════════════════════
#  2D 占据栅格地图生成
# ═══════════════════════════════════════════════════════════

def generate_occupancy_grid(data, resolution=0.05,
                            z_min=-0.8, z_max=2.0,
                            min_points=2, free_thick=0,
                            use_downsample=True, voxel_size=0.05):
    """
    参数:
        data:       read_pcd 返回的字段字典
        resolution: 栅格分辨率 (m/pixel), 默认 0.05
        z_min:      高度下限 (过滤地面以下)
        z_max:      高度上限 (过滤天花板以上)
        min_points: 判定为占据的最小点数
        free_thick: 膨胀层厚度 (像素), 0 表示不膨胀
        use_downsample: 是否降采样
        voxel_size: 降采样体素大小
    返回:
        grid:       占据栅格 (0=空闲, 100=占据, -1=未知)
        origin:     [x, y, yaw] 地图原点
        resolution: 分辨率
    """
    x = data.get('x', data.get('X'))
    y = data.get('y', data.get('Y'))
    z = data.get('z', data.get('Z'))

    if x is None or y is None:
        raise ValueError("PCD 文件中未找到 x/y 字段")

    # 高度过滤
    if z is not None:
        mask = (z >= z_min) & (z <= z_max)
        x = x[mask]
        y = y[mask]
        z = z[mask]

    if len(x) == 0:
        raise ValueError("高度过滤后无有效点, 请调整 z_min/z_max")

    print(f"高度过滤后点数: {len(x)}")
    
    # 降采样
    if use_downsample and len(x) > 10000:
        print(f"降采样中 (体素大小: {voxel_size}m)...")
        x, y, z = voxel_downsample(x, y, z, voxel_size)
        print(f"降采样后点数: {len(x)}")

    print(f"有效点数: {len(x)}")
    print(f"X 范围: [{x.min():.2f}, {x.max():.2f}]")
    print(f"Y 范围: [{y.min():.2f}, {y.max():.2f}]")
    if z is not None:
        print(f"Z 范围: [{z.min():.2f}, {z.max():.2f}]")

    # 网格坐标映射
    x_min, x_max = x.min(), x.max()
    y_min, y_max = y.min(), y.max()

    width = int(np.ceil((x_max - x_min) / resolution)) + 1
    height = int(np.ceil((y_max - y_min) / resolution)) + 1

    print(f"栅格尺寸: {width} × {height} ({(width * height) / 1e6:.2f}M cells)")

    # 统计每个栅格内的点数
    xi = np.floor((x - x_min) / resolution).astype(np.int32)
    yi = np.floor((y - y_min) / resolution).astype(np.int32)

    # 过滤越界索引
    valid = (xi >= 0) & (xi < width) & (yi >= 0) & (yi < height)
    xi, yi = xi[valid], yi[valid]

    counts = np.zeros((height, width), dtype=np.int32)
    np.add.at(counts, (yi, xi), 1)

    # 生成占据栅格: >= min_points → 占据(100), 否则空闲(0)
    grid = np.full((height, width), -1, dtype=np.int8)  # 默认未知
    grid[counts >= min_points] = 100  # 占据
    grid[counts < min_points] = 0     # 空闲
    
    # 统计信息
    occupied = (grid == 100).sum()
    free = (grid == 0).sum()
    unknown = (grid == -1).sum()
    print(f"占据: {occupied} 格 ({occupied/(width*height)*100:.1f}%)")
    print(f"空闲: {free} 格 ({free/(width*height)*100:.1f}%)")
    print(f"未知: {unknown} 格 ({unknown/(width*height)*100:.1f}%)")

    # 可选膨胀层 (让障碍物更厚)
    if free_thick > 0:
        print(f"膨胀处理 (厚度: {free_thick} 像素)...")
        from scipy.ndimage import binary_dilation
        occupied_mask = grid == 100
        occupied_mask = binary_dilation(occupied_mask, iterations=free_thick)
        grid[occupied_mask] = 100
        
        # 膨胀后重新统计
        occupied = (grid == 100).sum()
        print(f"膨胀后占据: {occupied} 格 ({occupied/(width*height)*100:.1f}%)")

    # 地图原点 (左下角, ROS 坐标系)
    origin = [x_min, y_min, 0.0]

    return grid, origin, resolution


# ═══════════════════════════════════════════════════════════
#  保存为 PGM + YAML
# ═══════════════════════════════════════════════════════════

def save_map(grid, origin, resolution, output_dir, map_name):
    """
    保存占据栅格地图为 Nav2 兼容格式:
      - map_name.pgm  (PGM 灰度图像)
      - map_name.yaml (元数据)
    """
    os.makedirs(output_dir, exist_ok=True)

    pgm_path = os.path.join(output_dir, f"{map_name}.pgm")
    yaml_path = os.path.join(output_dir, f"{map_name}.yaml")

    # 生成 PGM (0=空闲/白色, 100=占据/黑色, -1=未知/灰色)
    img = np.full_like(grid, 205, dtype=np.uint8)  # 未知: 灰色 205
    img[grid == 0] = 254                           # 空闲: 白色 254
    img[grid == 100] = 0                            # 占据: 黑色 0

    # PGM 格式: 行=height, 列=width, Y 轴翻转 (ROS 约定)
    img = np.flipud(img)

    from PIL import Image
    pil_img = Image.fromarray(img, mode='L')
    pil_img.save(pgm_path)

    # 写 YAML
    yaml_content = f"""image: {os.path.basename(pgm_path)}
mode: trinary
resolution: {resolution}
origin: [{origin[0]}, {origin[1]}, {origin[2]}]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.25
"""

    with open(yaml_path, 'w') as f:
        f.write(yaml_content)

    print(f"\n导航地图已生成:")
    print(f"  PGM: {pgm_path}")
    print(f"  YAML: {yaml_path}")
    print(f"  栅格: {grid.shape[1]} × {grid.shape[0]}")
    print(f"  分辨率: {resolution} m/pixel")

    return pgm_path, yaml_path


# ═══════════════════════════════════════════════════════════
#  入口
# ═══════════════════════════════════════════════════════════

def main():
    parser = argparse.ArgumentParser(description='MID360 SLAM 导航地图生成器 (改进版)')
    parser.add_argument('pcd_file', nargs='?',
                        default=os.path.expanduser('~/mid360_map/map.pcd'),
                        help='PCD 点云文件路径')
    parser.add_argument('-o', '--output', default=os.path.expanduser('~/mid360_map'),
                        help='输出目录')
    parser.add_argument('-n', '--name', default=None,
                        help='地图名称 (默认: navmap_时间戳)')
    parser.add_argument('-r', '--resolution', type=float, default=0.05,
                        help='栅格分辨率 (m/pixel), 默认 0.05')
    parser.add_argument('--zmin', type=float, default=-0.1,
                        help='高度下限 (m), 默认 -0.1')
    parser.add_argument('--zmax', type=float, default=2.0,
                        help='高度上限 (m), 默认 2.0')
    parser.add_argument('--min-points', type=int, default=3,
                        help='判定为占据的最小点数, 默认 3')
    parser.add_argument('--inflate', type=int, default=2,
                        help='膨胀层厚度 (像素), 默认 2')
    parser.add_argument('--no-downsample', action='store_true',
                        help='不进行降采样')
    parser.add_argument('--voxel-size', type=float, default=0.05,
                        help='降采样体素大小 (m), 默认 0.05')

    args = parser.parse_args()

    if not os.path.exists(args.pcd_file):
        print(f"错误: 文件不存在: {args.pcd_file}")
        sys.exit(1)

    map_name = args.name or f"navmap_{datetime.now().strftime('%Y%m%d_%H%M%S')}"

    print(f"输入: {args.pcd_file}")
    print(f"分辨率: {args.resolution} m/pixel")
    print(f"高度过滤: [{args.zmin}, {args.zmax}] m")
    print(f"最小占据点: {args.min_points}")
    print(f"膨胀厚度: {args.inflate} 像素")
    print(f"降采样: {'否' if args.no_downsample else '是'}")
    print()

    data, total = read_pcd(args.pcd_file)
    print(f"总点数: {total}")

    grid, origin, resolution = generate_occupancy_grid(
        data,
        resolution=args.resolution,
        z_min=args.zmin,
        z_max=args.zmax,
        min_points=args.min_points,
        free_thick=args.inflate,
        use_downsample=not args.no_downsample,
        voxel_size=args.voxel_size
    )

    pgm_path, yaml_path = save_map(grid, origin, resolution, args.output, map_name)

    print(f"\n完成! 可运行以下命令测试地图:")
    print(f"  ros2 run nav2_map_server map_server --ros-args -p yaml_filename:={yaml_path}")

if __name__ == '__main__':
    main()