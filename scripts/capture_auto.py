#!/usr/bin/env python3
"""
点云捕获工具 - 自动捕获点云并生成导航地图
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
import os
import time
from datetime import datetime


class PointCloudCapture(Node):
    def __init__(self):
        super().__init__('pointcloud_capture')
        
        # 订阅原始点云
        self.subscription = self.create_subscription(
            PointCloud2,
            '/livox/lidar',
            self.cloud_callback,
            10
        )
        
        # 累积的点云数据
        self.accumulated_points = []
        self.frame_count = 0
        self.capture_duration = 10  # 默认采集10秒
        self.start_time = None
        self.is_capturing = False
        
        self.get_logger().info('点云捕获工具已启动')
        self.get_logger().info('开始捕获点云，持续10秒...')
        
        # 自动开始捕获
        self.start_time = time.time()
        self.is_capturing = True
        
    def cloud_callback(self, msg):
        """点云回调函数"""
        if not self.is_capturing:
            return
            
        elapsed = time.time() - self.start_time
        if elapsed > self.capture_duration:
            if self.is_capturing:
                self.is_capturing = False
                self.get_logger().info(f'捕获完成，共 {self.frame_count} 帧，{len(self.accumulated_points)} 个点')
                self.save_pcd()
            return
            
        # 解析点云数据
        points = []
        for p in pc2.read_points(msg, skip_nans=True):
            points.append([p[0], p[1], p[2]])
            
        self.accumulated_points.extend(points)
        self.frame_count += 1
        
        if self.frame_count % 10 == 0:
            self.get_logger().info(f'已捕获 {self.frame_count} 帧，{len(self.accumulated_points)} 个点，剩余 {self.capture_duration - elapsed:.1f} 秒')
            
    def save_pcd(self):
        """保存点云为PCD文件"""
        if len(self.accumulated_points) == 0:
            self.get_logger().error('没有捕获到点云数据')
            return
            
        # 创建输出目录
        output_dir = os.path.expanduser('~/mid360_map/navmap')
        os.makedirs(output_dir, exist_ok=True)
        
        # 生成文件名（使用时间戳）
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        pcd_file = os.path.join(output_dir, f'{timestamp}.pcd')
        
        # 转换为numpy数组
        points = np.array(self.accumulated_points, dtype=np.float32)
        
        # 降采样（如果点太多）
        if len(points) > 1000000:
            self.get_logger().info(f'点云过大 ({len(points)} 点)，进行降采样...')
            indices = np.random.choice(len(points), 1000000, replace=False)
            points = points[indices]
            
        # 保存PCD文件
        with open(pcd_file, 'w') as f:
            f.write('# .PCD v0.7 - Point Cloud Data file format\n')
            f.write('VERSION 0.7\n')
            f.write('FIELDS x y z\n')
            f.write('SIZE 4 4 4\n')
            f.write('TYPE F F F\n')
            f.write('COUNT 1 1 1\n')
            f.write(f'WIDTH {len(points)}\n')
            f.write('HEIGHT 1\n')
            f.write('VIEWPOINT 0 0 0 1 0 0 0\n')
            f.write(f'POINTS {len(points)}\n')
            f.write('DATA ascii\n')
            
            for p in points:
                f.write(f'{p[0]:.6f} {p[1]:.6f} {p[2]:.6f}\n')
                
        self.get_logger().info(f'点云已保存到: {pcd_file}')
        self.get_logger().info(f'点数: {len(points)}')
        self.get_logger().info('')
        self.get_logger().info('正在自动生成导航地图...')
        
        # 自动生成导航地图
        import subprocess
        script_path = os.path.expanduser('~/mid360_slam_ws/scripts/generate_navmap.py')
        navmap_dir = os.path.join(output_dir, f'navmap_{timestamp}')
        result = subprocess.run(
            ['python3', script_path, pcd_file, '-o', navmap_dir],
            capture_output=True,
            text=True
        )
        
        if result.returncode == 0:
            self.get_logger().info('导航地图生成成功!')
            self.get_logger().info(result.stdout)
            
            # 重命名生成的文件为时间戳格式
            import glob
            import shutil
            navmap_files = glob.glob(os.path.join(navmap_dir, '*.yaml'))
            if navmap_files:
                old_yaml = navmap_files[0]
                new_yaml = os.path.join(output_dir, f'{timestamp}.yaml')
                old_pgm = old_yaml.replace('.yaml', '.pgm')
                new_pgm = os.path.join(output_dir, f'{timestamp}.pgm')
                
                # 复制并重命名文件
                shutil.copy2(old_yaml, new_yaml)
                shutil.copy2(old_pgm, new_pgm)
                
                self.get_logger().info(f'导航地图已保存到:')
                self.get_logger().info(f'  PGM: {new_pgm}')
                self.get_logger().info(f'  YAML: {new_yaml}')
                
                # 删除临时目录
                shutil.rmtree(navmap_dir)
        else:
            self.get_logger().error(f'导航地图生成失败: {result.stderr}')
            
        # 标记完成，让主循环退出
        self.capture_done = True


def main(args=None):
    rclpy.init(args=args)
    
    node = PointCloudCapture()
    node.capture_done = False
    
    try:
        while rclpy.ok() and not node.capture_done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
