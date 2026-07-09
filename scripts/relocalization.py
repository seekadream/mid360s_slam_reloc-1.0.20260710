#!/usr/bin/env python3
"""
重定位节点
功能：根据已建好的地图，实时定位雷达在地图中的位置
使用ICP算法将实时点云与地图匹配
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import TransformBroadcaster
import numpy as np
import struct
import math
import time
from collections import deque


class RelocalizationNode(Node):
    def __init__(self):
        super().__init__('relocalization')
        
        # 参数声明
        self.declare_parameter('map_path', '/home/pi/mid360_map/map.pcd')
        self.declare_parameter('map_origin_x', -38.778350830078125)
        self.declare_parameter('map_origin_y', -12.73618221282959)
        self.declare_parameter('map_origin_z', 0.0)
        self.declare_parameter('icp_max_iterations', 50)
        self.declare_parameter('icp_threshold', 0.05)
        self.declare_parameter('icp_max_distance', 1.0)
        self.declare_parameter('downsample_size', 0.1)
        
        # 获取参数
        self.map_path = self.get_parameter('map_path').value
        self.map_origin_x = self.get_parameter('map_origin_x').value
        self.map_origin_y = self.get_parameter('map_origin_y').value
        self.map_origin_z = self.get_parameter('map_origin_z').value
        self.icp_max_iterations = self.get_parameter('icp_max_iterations').value
        self.icp_threshold = self.get_parameter('icp_threshold').value
        self.icp_max_distance = self.get_parameter('icp_max_distance').value
        self.downsample_size = self.get_parameter('downsample_size').value
        
        # 加载地图
        self.map_points = None
        self.load_map()
        
        # 当前位姿 (相对于地图原点)
        self.current_pose = np.eye(4)
        self.pose_initialized = False
        
        # 位姿历史
        self.pose_history = deque(maxlen=100)
        
        # 发布器
        self.pose_pub = self.create_publisher(PoseStamped, '/relocalization_pose', 10)
        self.aligned_cloud_pub = self.create_publisher(PointCloud2, '/aligned_cloud', 10)
        self.map_cloud_pub = self.create_publisher(PointCloud2, '/map_cloud', 10)
        
        # TF广播器
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # 订阅实时点云
        self.cloud_sub = self.create_subscription(
            PointCloud2,
            '/cloud_registered',
            self.cloud_callback,
            10
        )
        
        # 定时器，发布TF和地图
        self.tf_timer = self.create_timer(0.05, self.publish_tf)
        self.map_timer = self.create_timer(1.0, self.publish_map_cloud)  # 每秒发布一次地图
        
        self.get_logger().info('重定位节点已启动')
        self.get_logger().info(f'地图路径: {self.map_path}')
        self.get_logger().info(f'地图原点: ({self.map_origin_x:.2f}, {self.map_origin_y:.2f}, {self.map_origin_z:.2f})')
        
        if self.map_points is not None:
            self.get_logger().info(f'地图点数: {len(self.map_points)}')
        else:
            self.get_logger().warn('地图未加载！')
    
    def load_map(self):
        """加载PCD地图文件"""
        try:
            self.get_logger().info(f'正在加载地图: {self.map_path}')
            
            with open(self.map_path, 'rb') as f:
                # 解析PCD文件头
                header = {}
                while True:
                    line = f.readline().decode('utf-8', errors='ignore').strip()
                    if line.startswith('DATA'):
                        header['data'] = line.split()[1]
                        break
                    parts = line.split()
                    if len(parts) >= 2:
                        header[parts[0]] = parts[1:]
                
                # 获取点数和字段
                points_num = int(header.get('POINTS', [0])[0])
                fields = header.get('FIELDS', ['x', 'y', 'z'])
                sizes = header.get('SIZE', ['4', '4', '4'])
                types = header.get('TYPE', ['F', 'F', 'F'])
                
                self.get_logger().info(f'地图点数: {points_num}, 字段: {fields}')
                
                # 读取点云数据
                if header['data'] == 'ascii':
                    points = []
                    for _ in range(points_num):
                        line = f.readline().decode('utf-8').strip()
                        if line:
                            parts = line.split()
                            if len(parts) >= 3:
                                x, y, z = float(parts[0]), float(parts[1]), float(parts[2])
                                if abs(x) < 100 and abs(y) < 100 and abs(z) < 100:
                                    points.append([x, y, z])
                    self.map_points = np.array(points, dtype=np.float32)
                elif header['data'] == 'binary':
                    # 二进制格式
                    point_step = sum([int(s) for s in sizes[:3]])
                    data = f.read(points_num * point_step)
                    points = []
                    for i in range(points_num):
                        offset = i * point_step
                        x = struct.unpack_from('f', data, offset)[0]
                        y = struct.unpack_from('f', data, offset + 4)[0]
                        z = struct.unpack_from('f', data, offset + 8)[0]
                        if abs(x) < 100 and abs(y) < 100 and abs(z) < 100:
                            points.append([x, y, z])
                    self.map_points = np.array(points, dtype=np.float32)
                
                # 降采样
                if self.map_points is not None and len(self.map_points) > 0:
                    self.map_points = self.voxel_downsample(self.map_points, self.downsample_size)
                    self.get_logger().info(f'地图加载完成，降采样后点数: {len(self.map_points)}')
                    
                    # 计算地图边界
                    min_coords = self.map_points.min(axis=0)
                    max_coords = self.map_points.max(axis=0)
                    self.get_logger().info(f'地图范围: X[{min_coords[0]:.1f}, {max_coords[0]:.1f}], '
                                          f'Y[{min_coords[1]:.1f}, {max_coords[1]:.1f}], '
                                          f'Z[{min_coords[2]:.1f}, {max_coords[2]:.1f}]')
                    
        except Exception as e:
            self.get_logger().error(f'加载地图失败: {e}')
            self.map_points = None
    
    def voxel_downsample(self, points, voxel_size):
        """体素降采样"""
        if len(points) == 0:
            return points
        
        min_coords = points.min(axis=0)
        voxel_indices = np.floor((points - min_coords) / voxel_size).astype(int)
        
        voxel_dict = {}
        for i, idx in enumerate(voxel_indices):
            key = tuple(idx)
            if key not in voxel_dict:
                voxel_dict[key] = []
            voxel_dict[key].append(i)
        
        downsampled = []
        for indices in voxel_dict.values():
            center = points[indices].mean(axis=0)
            downsampled.append(center)
        
        return np.array(downsampled, dtype=np.float32)
    
    def cloud_callback(self, msg):
        """处理实时点云"""
        if self.map_points is None:
            return
        
        try:
            # 解析点云
            points = self.pointcloud2_to_xyz(msg)
            if points is None or len(points) < 100:
                return
            
            # 降采样
            points = self.voxel_downsample(points, self.downsample_size)
            
            # ICP匹配
            if not self.pose_initialized:
                # 首次使用初始位姿
                self.current_pose = np.eye(4)
                self.pose_initialized = True
                self.get_logger().info('初始化位姿')
            
            # 执行ICP
            transform, fitness = self.icp(points, self.map_points, self.current_pose)
            
            if fitness > 0.3:  # 匹配度阈值
                self.current_pose = transform
                self.pose_history.append(transform.copy())
                
                # 发布局部位姿
                self.publish_pose(transform, fitness)
                
                # 发布对齐后的点云
                self.publish_aligned_cloud(points, transform, msg.header)
                
                # 输出状态
                if len(self.pose_history) % 10 == 0:
                    x, y, z = self.get_position(transform)
                    yaw = self.get_yaw(transform)
                    self.get_logger().info(
                        f'位置: ({x:.2f}, {y:.2f}, {z:.2f})m, '
                        f'朝向: {math.degrees(yaw):.1f}°, '
                        f'匹配度: {fitness:.2f}'
                    )
            else:
                self.get_logger().warn(f'匹配失败，匹配度: {fitness:.2f}')
                
        except Exception as e:
            self.get_logger().error(f'处理点云错误: {e}')
    
    def icp(self, source, target, initial_transform=None):
        """ICP算法实现"""
        if initial_transform is None:
            current_transform = np.eye(4)
        else:
            current_transform = initial_transform.copy()
        
        # 将源点云变换到目标坐标系
        source_transformed = self.transform_points(source, current_transform)
        
        best_fitness = 0.0
        best_transform = current_transform.copy()
        
        for iteration in range(self.icp_max_iterations):
            # 找最近点
            matched_source, matched_target, fitness = self.find_correspondences(
                source_transformed, target
            )
            
            if fitness < 0.1:
                break
            
            # 计算变换矩阵
            if len(matched_source) >= 3:
                delta_transform = self.compute_transform(matched_source, matched_target)
                current_transform = delta_transform @ current_transform
                source_transformed = self.transform_points(source, current_transform)
                
                best_fitness = fitness
                best_transform = current_transform.copy()
                
                # 检查收敛
                delta_translation = np.linalg.norm(delta_transform[:3, 3])
                delta_rotation = np.arccos(
                    np.clip((np.trace(delta_transform[:3, :3]) - 1) / 2, -1, 1)
                )
                
                if delta_translation < self.icp_threshold and delta_rotation < 0.01:
                    break
        
        return best_transform, best_fitness
    
    def find_correspondences(self, source, target):
        """查找最近点对应关系"""
        matched_source = []
        matched_target = []
        
        # 使用KD树加速查找（简化版本使用暴力搜索）
        for src_point in source:
            # 计算到所有目标点的距离
            distances = np.linalg.norm(target - src_point, axis=1)
            min_idx = np.argmin(distances)
            min_dist = distances[min_idx]
            
            if min_dist < self.icp_max_distance:
                matched_source.append(src_point)
                matched_target.append(target[min_idx])
        
        matched_source = np.array(matched_source, dtype=np.float32)
        matched_target = np.array(matched_target, dtype=np.float32)
        
        fitness = len(matched_source) / len(source) if len(source) > 0 else 0.0
        
        return matched_source, matched_target, fitness
    
    def compute_transform(self, source, target):
        """计算最优变换矩阵"""
        # 计算质心
        src_centroid = source.mean(axis=0)
        tgt_centroid = target.mean(axis=0)
        
        # 去中心化
        src_centered = source - src_centroid
        tgt_centered = target - tgt_centroid
        
        # SVD分解
        H = src_centered.T @ tgt_centered
        U, S, Vt = np.linalg.svd(H)
        
        # 计算旋转矩阵
        R = Vt.T @ U.T
        
        # 处理反射情况
        if np.linalg.det(R) < 0:
            Vt[-1, :] *= -1
            R = Vt.T @ U.T
        
        # 计算平移向量
        t = tgt_centroid - R @ src_centroid
        
        # 构建变换矩阵
        transform = np.eye(4)
        transform[:3, :3] = R
        transform[:3, 3] = t
        
        return transform
    
    def transform_points(self, points, transform):
        """变换点云"""
        R = transform[:3, :3]
        t = transform[:3, 3]
        return (R @ points.T).T + t
    
    def get_position(self, transform):
        """提取位置"""
        return transform[0, 3], transform[1, 3], transform[2, 3]
    
    def get_yaw(self, transform):
        """提取偏航角"""
        R = transform[:3, :3]
        return math.atan2(R[1, 0], R[0, 0])
    
    def pointcloud2_to_xyz(self, msg):
        """将PointCloud2转换为xyz坐标数组"""
        try:
            fields = msg.fields
            point_step = msg.point_step
            num_points = msg.width * msg.height
            
            x_offset = None
            y_offset = None
            z_offset = None
            
            for field in fields:
                if field.name == 'x':
                    x_offset = field.offset
                elif field.name == 'y':
                    y_offset = field.offset
                elif field.name == 'z':
                    z_offset = field.offset
            
            if x_offset is None or y_offset is None or z_offset is None:
                return None
            
            points = []
            data = msg.data
            
            for i in range(num_points):
                base_idx = i * point_step
                x = struct.unpack_from('f', data, base_idx + x_offset)[0]
                y = struct.unpack_from('f', data, base_idx + y_offset)[0]
                z = struct.unpack_from('f', data, base_idx + z_offset)[0]
                
                if abs(x) < 100 and abs(y) < 100 and abs(z) < 100:
                    points.append([x, y, z])
            
            return np.array(points, dtype=np.float32) if points else None
            
        except Exception as e:
            self.get_logger().error(f'点云转换错误: {e}')
            return None
    
    def publish_pose(self, transform, fitness):
        """发布位姿"""
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'map_origin'
        
        # 位置
        pose_msg.pose.position.x = transform[0, 3]
        pose_msg.pose.position.y = transform[1, 3]
        pose_msg.pose.position.z = transform[2, 3]
        
        # 四元数
        R = transform[:3, :3]
        qw = math.sqrt(1 + R[0, 0] + R[1, 1] + R[2, 2]) / 2
        qx = (R[2, 1] - R[1, 2]) / (4 * qw)
        qy = (R[0, 2] - R[2, 0]) / (4 * qw)
        qz = (R[1, 0] - R[0, 1]) / (4 * qw)
        
        pose_msg.pose.orientation.x = qx
        pose_msg.pose.orientation.y = qy
        pose_msg.pose.orientation.z = qz
        pose_msg.pose.orientation.w = qw
        
        self.pose_pub.publish(pose_msg)
    
    def publish_aligned_cloud(self, points, transform, header):
        """发布对齐后的点云"""
        # 变换点云
        points_transformed = self.transform_points(points, transform)
        
        # 创建PointCloud2消息
        msg = PointCloud2()
        msg.header = header
        msg.header.frame_id = 'map_origin'
        msg.height = 1
        msg.width = len(points_transformed)
        
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        
        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = msg.point_step * msg.width
        msg.is_dense = True
        msg.data = points_transformed.astype(np.float32).tobytes()
        
        self.aligned_cloud_pub.publish(msg)
    
    def publish_map_cloud(self):
        """发布地图点云"""
        if self.map_points is None:
            return
        
        try:
            msg = PointCloud2()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'map_origin'
            msg.height = 1
            msg.width = len(self.map_points)
            
            msg.fields = [
                PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            ]
            
            msg.is_bigendian = False
            msg.point_step = 12
            msg.row_step = msg.point_step * msg.width
            msg.is_dense = True
            msg.data = self.map_points.astype(np.float32).tobytes()
            
            self.map_cloud_pub.publish(msg)
        except Exception as e:
            self.get_logger().error(f'发布地图点云错误: {e}')
    
    def publish_tf(self):
        """发布TF变换"""
        if not self.pose_initialized:
            return
        
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map_origin'
        t.child_frame_id = 'relocalization_body'
        
        # 位置
        t.transform.translation.x = self.current_pose[0, 3]
        t.transform.translation.y = self.current_pose[1, 3]
        t.transform.translation.z = self.current_pose[2, 3]
        
        # 四元数
        R = self.current_pose[:3, :3]
        qw = math.sqrt(1 + R[0, 0] + R[1, 1] + R[2, 2]) / 2
        qx = (R[2, 1] - R[1, 2]) / (4 * qw)
        qy = (R[0, 2] - R[2, 0]) / (4 * qw)
        qz = (R[1, 0] - R[0, 1]) / (4 * qw)
        
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = RelocalizationNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
