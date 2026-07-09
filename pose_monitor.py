#!/usr/bin/env python3
"""实时输出位姿 (x,y,z,yaw,roll,pitch) 和定位精度因子"""
import os
import math
import signal
import sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from datetime import datetime

LOG_FILE = os.path.expanduser("~/Desktop/pose_log.txt")

HEADER_NOTE = (
    "字段说明:\n"
    "  HH:MM:SS.mmm - 时间(时:分:秒.毫秒)\n"
    "  X/Y/Z        - 位置坐标 (米)\n"
    "  Acc          - 定位精度 (米)\n"
    "  Grade        - 精度等级 (A<0.1m B<0.5m C<1.0m D>=1.0m)\n"
    "  Yaw          - 航向角 (弧度), 即绕Z轴旋转\n"
    "  Roll         - 横滚角 (弧度), 即绕X轴旋转\n"
    "  Pitch        - 俯仰角 (弧度), 即绕Y轴旋转\n"
    "  Qx/Qy/Qz/Qw  - 四元数姿态\n"
)

ACC_NOTE = (
    "精度等级说明:\n"
    "  A: 精度<0.1m  (精度很高,适合高精度重定位)\n"
    "  B: 精度<0.5m  (精度较好,满足大部分场景)\n"
    "  C: 精度<1.0m  (精度一般,可尝试重新初始化)\n"
    "  D: 精度>=1.0m (精度较差,建议重新建图或调整参数)\n"
    "\n"
    "角度说明:\n"
    "  Yaw(航向)   : 绕Z轴旋转角, 0°=朝X正方向\n"
    "  Roll(横滚)  : 绕X轴旋转角\n"
    "  Pitch(俯仰) : 绕Y轴旋转角\n"
)

def quat_to_euler(qx, qy, qz, qw):
    roll = math.atan2(2.0*(qw*qx + qy*qz), 1.0 - 2.0*(qx*qx + qy*qy))
    pitch = math.asin(max(-1.0, min(1.0, 2.0*(qw*qy - qz*qx))))
    yaw = math.atan2(2.0*(qw*qz + qx*qy), 1.0 - 2.0*(qy*qy + qz*qz))
    return roll, pitch, yaw

class PoseMonitor(Node):
    def __init__(self):
        super().__init__('pose_monitor')
        self.last_pose = None
        self.last_cov = None
        self.last_stamp = 0.0
        self.line_count = 0
        self.start_time = datetime.now()

        with open(LOG_FILE, 'w') as f:
            f.write(f"{'='*80}\n")
            f.write(f"程序启动: {self.start_time.strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}\n")
            f.write(f"{'='*80}\n")
            f.write(HEADER_NOTE)
            f.write(f"{'='*80}\n")
            f.write("# 时间(HH:MM:SS.mmm)  X(m)  Y(m)  Z(m)  "
                   "Acc(m)  Grade  Yaw(rad)  Roll(rad)  Pitch(rad)  "
                   "Qx  Qy  Qz  Qw\n")

        self.sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/localization/pose_with_covariance',
            self.callback, 10)

        self.timer = self.create_timer(0.1, self.timer_callback)

    def callback(self, msg):
        self.last_pose = msg.pose.pose
        self.last_cov = msg.pose.covariance
        self.last_stamp = msg.header.stamp

    def calc_accuracy(self, cov):
        if cov is None or len(cov) < 36:
            return 0.0
        vx = cov[0]
        vy = cov[7]
        vz = cov[14]
        rmse = math.sqrt((vx + vy + vz) / 3.0)
        return rmse

    def accuracy_grade(self, acc):
        if acc < 0.1:
            return "A"
        elif acc < 0.5:
            return "B"
        elif acc < 1.0:
            return "C"
        else:
            return "D"

    def timer_callback(self):
        now = datetime.now()
        ts = now.strftime('%H:%M:%S.%f')[:-3]
        if self.last_pose is None:
            print(f"[{ts}]  ⏳ 等待定位数据...")
            return

        p = self.last_pose.position
        o = self.last_pose.orientation
        roll, pitch, yaw = quat_to_euler(o.x, o.y, o.z, o.w)
        acc = self.calc_accuracy(self.last_cov)
        grade = self.accuracy_grade(acc)

        yaw_deg = math.degrees(yaw)
        roll_deg = math.degrees(roll)
        pitch_deg = math.degrees(pitch)

        print(f"[{ts}]  "
              f"X:{p.x:8.3f} Y:{p.y:8.3f} Z:{p.z:8.3f}  "
              f"Acc:{acc:.4f}[{grade}]  "
              f"Yaw:{yaw_deg:7.2f} Roll:{roll_deg:6.2f} Pitch:{pitch_deg:6.2f}")

        self.line_count += 1
        with open(LOG_FILE, 'a') as f:
            f.write(f"{ts} {p.x:.6f} {p.y:.6f} {p.z:.6f} "
                    f"{acc:.6f} [{grade}] "
                    f"{yaw:.6f} {roll:.6f} {pitch:.6f} "
                    f"{o.x:.6f} {o.y:.6f} {o.z:.6f} {o.w:.6f}\n")

NODE_INSTANCE = None

def sigterm_handler(signum, frame):
    global NODE_INSTANCE
    if NODE_INSTANCE:
        save_log(NODE_INSTANCE)
    rclpy.shutdown()
    sys.exit(0)

def save_log(node):
    end_time = datetime.now()
    duration = (end_time - node.start_time).total_seconds()
    with open(LOG_FILE, 'a') as f:
        f.write(f"\n{'='*80}\n")
        f.write(f"程序结束: {end_time.strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}\n")
        f.write(f"运行时长: {duration:.1f} 秒\n")
        f.write(f"总记录数: {node.line_count} 条\n")
        f.write(f"\n{ACC_NOTE}")
        f.write(f"{'='*80}\n")
    print(f"\n\n日志已保存: {LOG_FILE}")
    print(f"运行 {duration:.1f} 秒, 记录 {node.line_count} 条数据")

def main():
    global NODE_INSTANCE
    signal.signal(signal.SIGTERM, sigterm_handler)
    rclpy.init()
    node = PoseMonitor()
    NODE_INSTANCE = node
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        save_log(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
