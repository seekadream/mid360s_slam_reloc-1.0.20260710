#!/usr/bin/env python3
"""
键盘遥控节点
- w/s: 前进/后退
- a/d: 左移/右移
- q/e: 左转/右转
- 空格: 停止
- +/-: 调节速度
- x: 退出
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import threading
import select
import tty
import termios


class KeyboardTeleopNode(Node):
    def __init__(self):
        super().__init__('keyboard_teleop')
        
        # 发布速度话题
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # 参数
        self.max_vel = 0.5  # 最大速度 m/s
        self.speed_scale = 0.5  # 速度比例
        
        # 当前速度命令
        self.current_twist = Twist()
        
        self.get_logger().info('键盘遥控节点已启动')
        self.get_logger().info('使用方法:')
        self.get_logger().info('  w/s: 前进/后退')
        self.get_logger().info('  a/d: 左移/右移')
        self.get_logger().info('  q/e: 左转/右转')
        self.get_logger().info('  空格: 停止')
        self.get_logger().info('  +/-: 调节速度')
        self.get_logger().info('  x: 退出')
        
        # 定时器用于发布速度
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        # 检查是否在终端中运行
        self.is_tty = sys.stdin.isatty()
        
        if self.is_tty:
            self.get_logger().info('检测到终端，启用键盘控制')
            self.run_keyboard_control()
        else:
            self.get_logger().warn('未检测到终端，键盘控制不可用')
            self.get_logger().warn('请使用 RViz2 的 2D Goal Pose 设置导航目标')
    
    def run_keyboard_control(self):
        """运行键盘控制循环"""
        def keyboard_thread():
            fd = sys.stdin.fileno()
            old_settings = termios.tcgetattr(fd)
            try:
                tty.setraw(fd)
                while rclpy.ok():
                    if select.select([sys.stdin], [], [], 0.1)[0]:
                        char = sys.stdin.read(1)
                        
                        if char == 'w':
                            self.current_twist.linear.x = self.max_vel * self.speed_scale
                            self.get_logger().info(f'前进: {self.current_twist.linear.x:.2f} m/s')
                        elif char == 's':
                            self.current_twist.linear.x = -self.max_vel * self.speed_scale
                            self.get_logger().info(f'后退: {self.current_twist.linear.x:.2f} m/s')
                        elif char == 'a':
                            self.current_twist.linear.y = self.max_vel * self.speed_scale
                            self.get_logger().info(f'左移: {self.current_twist.linear.y:.2f} m/s')
                        elif char == 'd':
                            self.current_twist.linear.y = -self.max_vel * self.speed_scale
                            self.get_logger().info(f'右移: {self.current_twist.linear.y:.2f} m/s')
                        elif char == 'q':
                            self.current_twist.angular.z = self.max_vel * self.speed_scale * 2
                            self.get_logger().info(f'左转: {self.current_twist.angular.z:.2f} rad/s')
                        elif char == 'e':
                            self.current_twist.angular.z = -self.max_vel * self.speed_scale * 2
                            self.get_logger().info(f'右转: {self.current_twist.angular.z:.2f} rad/s')
                        elif char == ' ':
                            self.current_twist = Twist()
                            self.get_logger().info('停止')
                        elif char == '+' or char == '=':
                            self.speed_scale = min(1.0, self.speed_scale + 0.1)
                            self.get_logger().info(f'速度比例: {self.speed_scale:.1f}')
                        elif char == '-':
                            self.speed_scale = max(0.1, self.speed_scale - 0.1)
                            self.get_logger().info(f'速度比例: {self.speed_scale:.1f}')
                        elif char == 'x' or char == '\x03':  # x 或 Ctrl+C
                            self.current_twist = Twist()
                            self.get_logger().info('退出')
                            break
                        else:
                            # 松开按键时停止
                            self.current_twist = Twist()
                    
            finally:
                termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        
        # 启动键盘线程
        self.keyboard_thread = threading.Thread(target=keyboard_thread, daemon=True)
        self.keyboard_thread.start()
    
    def timer_callback(self):
        """定时发布速度命令"""
        self.cmd_vel_pub.publish(self.current_twist)


def main(args=None):
    rclpy.init(args=args)
    
    node = KeyboardTeleopNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 停止机器人
        twist = Twist()
        node.cmd_vel_pub.publish(twist)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
