#!/usr/bin/env python3
"""
雷达坐标监控节点
功能：实时输出雷达在地图坐标系中的位置
"""

import rclpy
from rclpy.node import Node
from tf2_ros import TransformListener, Buffer
from geometry_msgs.msg import TransformStamped
import math
import time


class CoordinateMonitor(Node):
    def __init__(self):
        super().__init__('coordinate_monitor')
        
        # TF2 缓冲区和监听器
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # 定时器，每0.5秒查询一次坐标
        self.timer = self.create_timer(0.5, self.timer_callback)
        
        # 上一次打印的时间
        self.last_print_time = 0
        self.print_interval = 1.0  # 每1秒打印一次详细信息
        
        self.get_logger().info('坐标监控节点已启动')
        self.get_logger().info('正在监控 camera_init -> body 的坐标变换...')
        print('=' * 60)
        print('雷达坐标监控系统')
        print('=' * 60)
    
    def timer_callback(self):
        try:
            # 查询最新的坐标变换
            now = rclpy.time.Time()
            transform = self.tf_buffer.lookup_transform(
                'camera_init',  # 目标坐标系
                'body',         # 源坐标系（雷达/机器人）
                now,
                timeout=rclpy.duration.Duration(seconds=0.1)
            )
            
            # 提取坐标
            x = transform.transform.translation.x
            y = transform.transform.translation.y
            z = transform.transform.translation.z
            
            # 提取四元数并转换为欧拉角
            qx = transform.transform.rotation.x
            qy = transform.transform.rotation.y
            qz = transform.transform.rotation.z
            qw = transform.transform.rotation.w
            
            # 四元数转欧拉角
            roll, pitch, yaw = self.quaternion_to_euler(qx, qy, qz, qw)
            
            # 计算距离原点的距离
            distance = math.sqrt(x*x + y*y + z*z)
            
            # 控制打印频率
            current_time = time.time()
            if current_time - self.last_print_time >= self.print_interval:
                self.last_print_time = current_time
                
                # 打印坐标信息
                print(f'\n[雷达坐标] X: {x:.3f}m, Y: {y:.3f}m, Z: {z:.3f}m')
                print(f'[朝向角度] Yaw: {math.degrees(yaw):.1f}°, Pitch: {math.degrees(pitch):.1f}°, Roll: {math.degrees(roll):.1f}°')
                print(f'[距原点距离] {distance:.3f}m')
                print('-' * 40)
                
                # 同时输出到ROS日志
                self.get_logger().info(
                    f'位置: ({x:.2f}, {y:.2f}, {z:.2f})m, '
                    f'朝向: {math.degrees(yaw):.1f}°, '
                    f'距离: {distance:.2f}m'
                )
                
        except Exception as e:
            # 静默处理TF查询失败（可能还没收到数据）
            pass
    
    def quaternion_to_euler(self, x, y, z, w):
        """四元数转欧拉角"""
        # Roll (x轴旋转)
        sinr_cosp = 2.0 * (w * x + y * z)
        cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        
        # Pitch (y轴旋转)
        sinp = 2.0 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)
        
        # Yaw (z轴旋转)
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        return roll, pitch, yaw


def main(args=None):
    rclpy.init(args=args)
    
    monitor = CoordinateMonitor()
    
    try:
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        print('\n\n坐标监控已停止')
    finally:
        monitor.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
