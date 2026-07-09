#!/usr/bin/env python3
"""
IMU 诊断工具 - 检查 /livox/imu 话题数据是否正常
"""
import sys
import os
import math
import rclpy
from rclpy.node import Node
from rclpy.timer import Timer
from sensor_msgs.msg import Imu

OUTPUT_FILE = os.path.expanduser("~/桌面/IMU诊断结果.txt")
TIMEOUT_SEC = 10


class ImuChecker(Node):
    def __init__(self):
        super().__init__('imu_checker')
        self.sub = self.create_subscription(Imu, '/livox/imu', self.callback, 10)
        self.count = 0
        self.max_count = 30
        self.acc_sum = [0.0, 0.0, 0.0]
        self.gyro_sum = [0.0, 0.0, 0.0]
        self.lines = []
        self.done = False
        hdr = "\033[0;34m=== MID360 IMU 诊断 (采样30条, 超时10秒) ===\033[0m\n" \
              f"{'#':>4s}  {'acc_x':>10s}  {'acc_y':>10s}  {'acc_z':>10s}  {'||acc||':>10s}  {'gyro_x':>10s}  {'gyro_y':>10s}  {'gyro_z':>10s}"
        print(hdr)
        self.lines.append("=== MID360 IMU 诊断 ===")
        self.lines.append(f"{'#':>4s}  {'acc_x':>10s}  {'acc_y':>10s}  {'acc_z':>10s}  {'||acc||':>10s}  {'gyro_x':>10s}  {'gyro_y':>10s}  {'gyro_z':>10s}")

        # 超时检测: 10秒后无数据则报错
        def timeout_cb():
            if self.count == 0 and not self.done:
                self.lines.append("\n[错误] 超时10秒未收到 IMU 数据!")
                self.lines.append("请先启动 LiDAR 驱动 (双击 MID360一体化)")
                self._save()
                print("\n\033[0;31m[错误] 超时10秒未收到 IMU 数据 /livox/imu\033[0m")
                print("\033[0;31m请先启动 LiDAR 驱动再运行此诊断\033[0m")
                self.done = True
                self.destroy_node()
                rclpy.shutdown()
        self.timeout_timer = self.create_timer(TIMEOUT_SEC, timeout_cb)

    def callback(self, msg: Imu):
        if self.count >= self.max_count:
            return
        self.count += 1
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

        color = "\033[0;32m" if 0.5 < norm < 15.0 else "\033[0;31m"
        line = f"{self.count:>4d}  {ax:>10.4f}  {ay:>10.4f}  {az:>10.4f}  {norm:>10.4f}  {gx:>10.4f}  {gy:>10.4f}  {gz:>10.4f}"
        self.lines.append(line)
        print(f"{color}{line}\033[0m")

        if self.count == self.max_count and not self.done:
            self.done = True
            n = self.max_count
            acc_avg = [v/n for v in self.acc_sum]
            gyro_avg = [v/n for v in self.gyro_sum]
            acc_norm = math.sqrt(acc_avg[0]**2 + acc_avg[1]**2 + acc_avg[2]**2)

            def put(s):
                print(s)
                self.lines.append(s)

            put(f"\n=== 诊断结果 ===")
            put(f"加速度均值: ({acc_avg[0]:.4f}, {acc_avg[1]:.4f}, {acc_avg[2]:.4f})  |acc|={acc_norm:.4f}")
            put(f"陀螺仪均值: ({gyro_avg[0]:.4f}, {gyro_avg[1]:.4f}, {gyro_avg[2]:.4f})")

            if 0.5 < acc_norm < 1.5:
                status = "[OK] 加速度量级 ~1.0g，单位 g，IMU 正常"
            elif 8.0 < acc_norm < 12.0:
                status = "[!] 加速度量级 ~10，单位 m/s²，IMU 正常"
            else:
                status = f"[!!] 加速度量级异常 ({acc_norm:.2f})"

            put(status)

            if abs(gyro_avg[0]) < 0.01 and abs(gyro_avg[1]) < 0.01 and abs(gyro_avg[2]) < 0.01:
                put("[OK] 陀螺仪接近零，静止状态正常")
            else:
                put("[!] 陀螺仪不接近零，可能正在运动")

            self._save()
            self.destroy_node()
            rclpy.shutdown()

    def _save(self):
        with open(OUTPUT_FILE, 'w') as f:
            f.write('\n'.join(self.lines))
        print(f"\n诊断结果已保存到: {OUTPUT_FILE}", flush=True)


def main():
    rclpy.init(args=sys.argv)
    node = ImuChecker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node.count > 0:
            node.lines.append(f"\n[中断] 已采样 {node.count} 条")
            node._save()
        else:
            node.lines.append("\n[中断] 未收到任何 IMU 数据")
            node._save()


if __name__ == '__main__':
    main()
