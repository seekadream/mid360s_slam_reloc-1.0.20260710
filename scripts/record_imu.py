#!/usr/bin/env python3
"""
IMU 数据持续记录器 - 运行期间全部保存
保存到: ~/桌面/IMU_记录_YYYYMMDD_HHMMSS.txt
"""
import sys
import os
import math
import rclpy
from rclpy.node import Node
from rclpy.timer import Timer
from sensor_msgs.msg import Imu
from datetime import datetime

OUTPUT_FILE = os.path.expanduser("~/桌面/IMU_记录_{}.txt".format(
    datetime.now().strftime("%Y%m%d_%H%M%S")))
SAVE_INTERVAL = 100   # 每100条刷盘一次
PRINT_INTERVAL = 10   # 每10条打印一次到终端
TIMEOUT_SEC = 10


class ImuRecorder(Node):
    def __init__(self):
        super().__init__('imu_recorder')
        self.sub = self.create_subscription(Imu, '/livox/imu', self.callback, 10)
        self.count = 0
        self.lines = []
        self.acc_sum = [0.0, 0.0, 0.0]
        self.gyro_sum = [0.0, 0.0, 0.0]
        self.start_time = None
        self.f = open(OUTPUT_FILE, 'w')
        self.f.write(f"# MID360 IMU 持续记录\n")
        self.f.write(f"# 开始时间: {datetime.now()}\n")
        self.f.write(f"# {'#':>6s}  {'时间(s)':>12s}  {'acc_x(g)':>10s}  {'acc_y(g)':>10s}  {'acc_z(g)':>10s}  {'||acc||':>10s}  {'gyro_x':>10s}  {'gyro_y':>10s}  {'gyro_z':>10s}\n")

        self.timeout_timer = self.create_timer(TIMEOUT_SEC, self._timeout_cb)

        print(f"\033[0;34mIMU 持续记录已启动\033[0m")
        print(f"保存到: {OUTPUT_FILE}")
        print(f"终端每 {PRINT_INTERVAL} 条显示一次, 文件保存全部数据")
        print(f"{'#':>6s}  {'时间(s)':>12s}  {'acc_x':>10s}  {'acc_y':>10s}  {'acc_z':>10s}  {'||acc||':>10s}  {'gyro_x':>10s}  {'gyro_y':>10s}  {'gyro_z':>10s}")
        print("按 Ctrl+C 停止")

    def _timeout_cb(self):
        if self.count == 0:
            print(f"\n\033[0;31m[错误] 超时 {TIMEOUT_SEC} 秒未收到 IMU 数据 /livox/imu\033[0m")
            print("\033[0;31m请先启动 LiDAR 驱动再运行此工具\033[0m")
            print("\033[0;31m按 Ctrl+C 退出\033[0m")
            self.timeout_timer.cancel()

    def callback(self, msg: Imu):
        self.count += 1
        stamp_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.start_time is None:
            self.start_time = stamp_sec

        elapsed = stamp_sec - self.start_time
        ax = msg.linear_acceleration.x
        ay = msg.linear_acceleration.y
        az = msg.linear_acceleration.z
        gx = msg.angular_velocity.x
        gy = msg.angular_velocity.y
        gz = msg.angular_velocity.z
        norm = math.sqrt(ax*ax + ay*ay + az*az)

        self.acc_sum[0] += ax
        self.acc_sum[1] += ay
        self.acc_sum[2] += az
        self.gyro_sum[0] += gx
        self.gyro_sum[1] += gy
        self.gyro_sum[2] += gz

        # 写入文件 (每条都写)
        self.f.write(f"{self.count:>6d}  {elapsed:>12.4f}  {ax:>10.6f}  {ay:>10.6f}  {az:>10.6f}  {norm:>10.6f}  {gx:>10.6f}  {gy:>10.6f}  {gz:>10.6f}\n")

        # 每 PRINT_INTERVAL 条打印到终端
        if self.count % PRINT_INTERVAL == 0:
            n = self.count
            acc_avg = [v/n for v in self.acc_sum]
            gyro_avg = [v/n for v in self.gyro_sum]
            acc_norm_avg = math.sqrt(acc_avg[0]**2 + acc_avg[1]**2 + acc_avg[2]**2)
            color = "\033[0;32m" if 0.5 < norm < 15.0 else "\033[0;31m"
            print(f"{color}{self.count:>6d}  {elapsed:>12.4f}  {ax:>10.4f}  {ay:>10.4f}  {az:>10.4f}  {norm:>10.4f}  {gx:>10.4f}  {gy:>10.4f}  {gz:>10.4f}\033[0m")

        # 定期刷盘
        if self.count % SAVE_INTERVAL == 0:
            self.f.flush()

    def finish(self):
        if self.count > 0:
            n = self.count
            acc_avg = [v/n for v in self.acc_sum]
            gyro_avg = [v/n for v in self.gyro_sum]
            acc_norm_avg = math.sqrt(acc_avg[0]**2 + acc_avg[1]**2 + acc_avg[2]**2)
            summary = [
                f"\n=== 统计 (共 {n} 条) ===",
                f"加速度均值: ({acc_avg[0]:.6f}, {acc_avg[1]:.6f}, {acc_avg[2]:.6f})  |acc|={acc_norm_avg:.6f}",
                f"陀螺仪均值: ({gyro_avg[0]:.6f}, {gyro_avg[1]:.6f}, {gyro_avg[2]:.6f})",
                f"结束时间: {datetime.now()}",
            ]
            self.f.write('\n'.join(summary) + '\n')
        self.f.close()
        print(f"\n已保存 {self.count} 条数据到: {OUTPUT_FILE}")


def main():
    rclpy.init(args=sys.argv)
    node = ImuRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.finish()


if __name__ == '__main__':
    main()
