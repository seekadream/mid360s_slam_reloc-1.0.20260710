#!/usr/bin/env python3
"""
雷达坐标记录器
功能：实时保存雷达坐标到桌面txt文件，文件名含时间戳，每次运行覆盖旧文件
"""

import rclpy
from rclpy.node import Node
from tf2_ros import TransformListener, Buffer
import math
import time
import os
import glob
from datetime import datetime


class CoordinateLogger(Node):
    def __init__(self):
        super().__init__('coordinate_logger')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(0.1, self.timer_callback)

        # 桌面路径
        self.desktop = os.path.expanduser('~/桌面')

        # 删除旧的坐标文件
        old_files = glob.glob(os.path.join(self.desktop, 'lidar_coord_*.txt'))
        for f in old_files:
            try:
                os.remove(f)
                print(f"已删除旧文件: {os.path.basename(f)}")
            except Exception:
                pass

        # 创建新文件（文件名含时间戳）
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.filename = f"lidar_coord_{timestamp}.txt"
        self.filepath = os.path.join(self.desktop, self.filename)

        self.fp = open(self.filepath, 'w', encoding='utf-8')
        self.fp.write(f"# 雷达坐标记录文件\n")
        self.fp.write(f"# 创建时间: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
        self.fp.write(f"# 格式: 时间戳(s) | X(m) | Y(m) | Z(m) | Yaw(度) | 距原点距离(m)\n")
        self.fp.write(f"# {'='*70}\n")
        self.fp.flush()

        self.start_time = time.time()
        self.count = 0
        self.get_logger().info(f'坐标记录已启动，保存到: {self.filepath}')

    def timer_callback(self):
        try:
            now = rclpy.time.Time()
            transform = self.tf_buffer.lookup_transform(
                'camera_init', 'body', now,
                timeout=rclpy.duration.Duration(seconds=0.1)
            )
            x = transform.transform.translation.x
            y = transform.transform.translation.y
            z = transform.transform.translation.z
            qx = transform.transform.rotation.x
            qy = transform.transform.rotation.y
            qz = transform.transform.rotation.z
            qw = transform.transform.rotation.w

            # 计算yaw
            siny_cosp = 2.0 * (qw * qz + qx * qy)
            cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
            yaw = math.degrees(math.atan2(siny_cosp, cosy_cosp))

            distance = math.sqrt(x * x + y * y + z * z)
            elapsed = time.time() - self.start_time
            self.count += 1

            # 写入文件
            self.fp.write(f"{elapsed:8.2f}  | {x:+9.4f} | {y:+9.4f} | {z:+9.4f} | {yaw:+7.1f} | {distance:8.4f}\n")
            self.fp.flush()

            # 每100条打印一次状态
            if self.count % 100 == 0:
                print(f"已记录 {self.count} 条 | X:{x:+.3f} Y:{y:+.3f} Z:{z:+.3f}")

        except Exception:
            pass

    def destroy_node(self):
        if self.fp:
            self.fp.write(f"\n# 记录结束，共 {self.count} 条数据\n")
            self.fp.close()
            print(f"\n坐标记录已保存: {self.filepath}")
            print(f"共记录 {self.count} 条数据")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CoordinateLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('\n坐标记录已停止')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
