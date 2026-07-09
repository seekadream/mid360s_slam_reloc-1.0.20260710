#!/usr/bin/env python3
# PCD 点云文件查看器 (使用 matplotlib + numpy)
import sys
import os
import struct
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def read_pcd(filepath):
    """读取 PCD 文件 (支持 ASCII 和 binary)"""
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

        if header['VERSION'] != '.7' and not header['VERSION'].startswith('0.'):
            raise ValueError(f"不支持的 PCD 版本: {header['VERSION']}")

        fields = header['FIELDS'].split()
        types = header['TYPE'].split()
        sizes = [int(s) for s in header['SIZE'].split()]
        counts = [int(c) for c in header['COUNT'].split()]
        width = int(header['WIDTH'])
        height = int(header['HEIGHT'])
        points = int(header['POINTS']) if 'POINTS' in header else width * height
        data_type = header['DATA']

        dtype_map = {'I': 'I', 'U': 'I', 'F': 'f'}
        fmt = '<' + ''.join(dtype_map.get(t, 'f') for t in types)
        point_size = sum(sizes)
        raw = f.read()

        if data_type == 'ascii':
            data = np.loadtxt(raw.decode('utf-8').strip().split('\n'))
        elif data_type == 'binary':
            data = np.frombuffer(raw[:points * point_size], dtype=np.float32).reshape(points, -1)
        else:
            raise ValueError(f"不支持的数据类型: {data_type}")

        x_idx = fields.index('x') if 'x' in fields else 0
        y_idx = fields.index('y') if 'y' in fields else 1
        z_idx = fields.index('z') if 'z' in fields else 2

        return data[:, x_idx], data[:, y_idx], data[:, z_idx], points

def main():
    filepath = sys.argv[1] if len(sys.argv) > 1 else os.path.expanduser('~/mid360_map/map.pcd')

    if not os.path.exists(filepath):
        print(f"文件不存在: {filepath}")
        sys.exit(1)

    print(f"正在加载: {filepath}")
    x, y, z, n_points = read_pcd(filepath)
    print(f"点数: {n_points}")

    # 下采样以提高渲染速度 (最多 200000 点)
    if n_points > 200000:
        step = n_points // 200000
        x, y, z = x[::step], y[::step], z[::step]
        print(f"下采样至: {len(x)} 点")

    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(x, y, z, s=0.1, c=z, cmap='viridis', alpha=0.6)
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title(os.path.basename(filepath))
    print("显示点云... 关闭窗口退出")
    plt.show()

if __name__ == '__main__':
    main()
