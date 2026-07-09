#!/usr/bin/env python3
"""
点云捕获工具 - 从点云查看器捕获点云并保存为PCD文件
用于生成导航地图
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
import struct
import os
import time
from datetime import datetime


class PointCloudCapture(Node):
    def __init__(self):
        super().__init__('pointcloud_capture')
        
        # 订阅点云（支持原始或滤波后的点云）
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
        self.get_logger().info('使用方法:')
        self.get_logger().info('  1. 运行点云查看器: ./run_viewer.sh')
        self.get_logger().info('  2. 在另一个终端运行此脚本')
        self.get_logger().info('  3. 等待采集完成')
        
    def start_capture(self, duration=10):
        """开始捕获点云"""
        self.capture_duration = duration
        self.start_time = time.time()
        self.is_capturing = True
        self.accumulated_points = []
        self.frame_count = 0
        self.get_logger().info(f'开始捕获点云，持续 {duration} 秒...')
        
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
        self.get_logger().info('接下来可以生成导航地图:')
        self.get_logger().info(f'  python3 ~/mid360_slam_ws/scripts/generate_navmap.py {pcd_file}')
        
        # 自动生成导航地图
        self.get_logger().info('')
        self.get_logger().info('正在自动生成导航地图...')
        
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
            navmap_files = glob.glob(os.path.join(navmap_dir, '*.yaml'))
            if navmap_files:
                old_yaml = navmap_files[0]
                new_yaml = os.path.join(output_dir, f'{timestamp}.yaml')
                old_pgm = old_yaml.replace('.yaml', '.pgm')
                new_pgm = os.path.join(output_dir, f'{timestamp}.pgm')
                
                # 复制并重命名文件
                import shutil
                shutil.copy2(old_yaml, new_yaml)
                shutil.copy2(old_pgm, new_pgm)
                
                self.get_logger().info(f'导航地图已保存到:')
                self.get_logger().info(f'  PGM: {new_pgm}')
                self.get_logger().info(f'  YAML: {new_yaml}')
                
                # 删除临时目录
                shutil.rmtree(navmap_dir)
        else:
            self.get_logger().error(f'导航地图生成失败: {result.stderr}')


def main(args=None):
    rclpy.init(args=args)
    
    node = PointCloudCapture()
    
    # 等待用户确认开始
    import threading
    def wait_for_input():
        try:
            input('\n按 Enter 键开始捕获点云 (持续10秒)...')
            node.start_capture(10)
        except EOFError:
            pass
            
    input_thread = threading.Thread(target=wait_for_input, daemon=True)
    input_thread.start()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
