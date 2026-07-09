#!/usr/bin/env python3
"""
MID360 SLAM 实时位姿输出 + 保存到桌面
订阅 /Odometry, 输出: x, y, z, 时间, 速度, 航向角
"""
import sys
import math
import time
import signal
import os
from datetime import datetime
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from nav_msgs.msg import Odometry
import threading


def quat_to_yaw(q):
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def color_red(s):
    return f"\033[0;31m{s}\033[0m"


def color_yellow(s):
    return f"\033[0;33m{s}\033[0m"


def color_green(s):
    return f"\033[0;32m{s}\033[0m"


def color_blue(s):
    return f"\033[0;34m{s}\033[0m"


class PosePrinter(Node):
    def __init__(self):
        super().__init__('pose_printer')
        
        # 使用可靠的QoS配置
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        
        self.sub = self.create_subscription(
            Odometry, '/Odometry', self.callback, qos_profile)

        self.last_ts = 0.0
        self.first_pos = None
        self.first_yaw = None
        self.recv_count = 0
        self.last_recv_time = time.time()
        self.running = True
        
        signal.signal(signal.SIGINT, self._sigint)
        signal.signal(signal.SIGTERM, self._sigint)

        # 桌面TXT日志
        desktop_path = os.path.expanduser('~/桌面')
        ts = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.txt_path = os.path.join(desktop_path, f'pose_log_{ts}.txt')
        self.txt_file = open(self.txt_path, 'w', encoding='utf-8')
        
        # 写入表头
        self.txt_file.write("=" * 80 + "\n")
        self.txt_file.write("MID360 SLAM 实时位姿日志\n")
        self.txt_file.write(f"开始时间: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
        self.txt_file.write("=" * 80 + "\n")
        self.txt_file.write(f"{'时间':>14s}  {'x(m)':>12s}  {'y(m)':>12s}  {'z(m)':>12s}  {'速度(m/s)':>10s}  {'航向(°)':>8s}  {'距离(m)':>10s}\n")
        self.txt_file.write("-" * 80 + "\n")
        self.txt_file.flush()

        print(color_blue("╔══════════════════════════════════════════════════════╗"))
        print(color_blue("║  MID360 SLAM 实时位姿                              ║"))
        print(color_blue("║  订阅: /Odometry                                  ║"))
        print(color_blue("╚══════════════════════════════════════════════════════╝"))
        print(f"日志文件: {self.txt_path}")
        print(f"{'时间':>14s}  {'x(m)':>12s}  {'y(m)':>12s}  {'z(m)':>12s}  {'速度(m/s)':>10s}  {'航向(°)':>8s}  {'距离(m)':>10s}")
        
        # 启动心跳检测线程
        self.heartbeat_thread = threading.Thread(target=self._heartbeat_check, daemon=True)
        self.heartbeat_thread.start()

    def _heartbeat_check(self):
        """心跳检测：如果超过5秒没有收到数据，打印警告"""
        while self.running:
            time.sleep(5)
            elapsed = time.time() - self.last_recv_time
            if elapsed > 5 and self.recv_count > 0:
                print(color_yellow(f"\n[警告] 已 {elapsed:.0f} 秒未收到 /Odometry 数据"))
                print(color_yellow("请检查 SLAM 系统是否正常运行"))
                self.txt_file.write(f"\n[警告] 已 {elapsed:.0f} 秒未收到数据\n")
                self.txt_file.flush()

    def _sigint(self, signum, frame):
        self.running = False

    def callback(self, msg: Odometry):
        self.recv_count += 1
        self.last_recv_time = time.time()
        
        stamp = msg.header.stamp
        t_sec = stamp.sec + stamp.nanosec * 1e-9

        # 每0.1秒输出一次
        if t_sec - self.last_ts < 0.1:
            return
        self.last_ts = t_sec

        t_str = time.strftime('%H:%M:%S', time.localtime(t_sec))
        t_str += f'.{stamp.nanosec // 1000000:03d}'

        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        yaw = quat_to_yaw(ori) * 180.0 / math.pi

        twist = msg.twist.twist
        vx = twist.linear.x
        vy = twist.linear.y
        vz = twist.linear.z
        v = math.sqrt(vx * vx + vy * vy + vz * vz)

        if self.first_pos is None:
            self.first_pos = (pos.x, pos.y, pos.z)
            self.first_yaw = yaw

        dx = pos.x - self.first_pos[0]
        dy = pos.y - self.first_pos[1]
        dz = pos.z - self.first_pos[2]
        dist = math.sqrt(dx * dx + dy * dy + dz * dz)

        # 控制台输出
        line = color_green(
            f"{t_str:>14s}  {pos.x:>12.3f}  {pos.y:>12.3f}  {pos.z:>12.3f}  "
            f"{v:>10.3f}  {yaw:>8.2f}  {dist:>10.3f}")
        print(line, flush=True)

        # 写入TXT文件
        txt_line = f"{t_str:>14s}  {pos.x:>12.6f}  {pos.y:>12.6f}  {pos.z:>12.6f}  {v:>10.6f}  {yaw:>8.4f}  {dist:>10.6f}\n"
        self.txt_file.write(txt_line)
        self.txt_file.flush()


def main():
    rclpy.init(args=sys.argv)
    node = PosePrinter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(color_red(f"\n错误: {e}"))
    finally:
        # 写入结束信息
        node.running = False
        node.txt_file.write("\n" + "=" * 80 + "\n")
        node.txt_file.write(f"结束时间: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
        node.txt_file.write(f"总共接收: {node.recv_count} 条消息\n")
        node.txt_file.write("=" * 80 + "\n")
        node.txt_file.close()
        print(f"\n位姿日志已保存: {node.txt_path}")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()