#!/usr/bin/env python3
"""
雷达坐标显示页面
功能：独立窗口实时显示雷达在地图中的坐标信息，包含轨迹小地图和位置变化量
"""

import rclpy
from rclpy.node import Node
from tf2_ros import TransformListener, Buffer
import math
import time
import threading
import tkinter as tk
import collections


class CoordinateDisplayNode(Node):
    def __init__(self, gui_callback):
        super().__init__('coordinate_display')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(0.05, self.timer_callback)
        self.gui_callback = gui_callback
        self.prev_x = 0.0
        self.prev_y = 0.0
        self.prev_z = 0.0
        self.prev_initialized = False
        self.last_update_time = time.time()
        self.trajectory = collections.deque(maxlen=2000)
        
        # 建图原点坐标 (从navmap.yaml中读取)
        self.map_origin_x = -38.778350830078125
        self.map_origin_y = -12.73618221282959
        self.map_origin_z = 0.0
        
        self.get_logger().info('坐标显示节点已启动')
        self.get_logger().info(f'建图原点: ({self.map_origin_x:.2f}, {self.map_origin_y:.2f}, {self.map_origin_z:.2f})')

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
            roll, pitch, yaw = self.quaternion_to_euler(qx, qy, qz, qw)
            distance = math.sqrt(x * x + y * y + z * z)

            # 计算与上一时刻的位置变化量
            if not self.prev_initialized:
                dx = 0.0
                dy = 0.0
                dz = 0.0
                self.prev_initialized = True
            else:
                dx = x - self.prev_x
                dy = y - self.prev_y
                dz = z - self.prev_z

            current_time = time.time()
            dt = current_time - self.last_update_time
            if dt > 0:
                speed = math.sqrt(dx * dx + dy * dy + dz * dz) / dt
            else:
                speed = 0.0

            self.prev_x = x
            self.prev_y = y
            self.prev_z = z
            self.last_update_time = current_time
            self.trajectory.append((x, y))

            self.gui_callback(x, y, z, roll, pitch, yaw, distance, speed, dx, dy, dz, list(self.trajectory), 
                            self.map_origin_x, self.map_origin_y, self.map_origin_z)
        except Exception:
            pass

    def quaternion_to_euler(self, x, y, z, w):
        sinr_cosp = 2.0 * (w * x + y * z)
        cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        sinp = 2.0 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return roll, pitch, yaw


class CoordinateDisplayGUI:
    def __init__(self):
        self.root = tk.Tk()
        self.root.title("MID360 雷达坐标监控")
        self.root.geometry("950x700")
        self.root.configure(bg='#1e1e2e')
        self.root.resizable(True, True)
        self._build_ui()

    def _build_ui(self):
        bg = '#1e1e2e'
        accent = '#89b4fa'
        green = '#a6e3a1'
        yellow = '#f9e2af'
        red = '#f38ba8'

        # 标题栏
        title_frame = tk.Frame(self.root, bg=bg)
        title_frame.pack(fill=tk.X, padx=15, pady=(10, 5))
        tk.Label(title_frame, text="MID360 雷达坐标监控系统",
                 font=('Helvetica', 18, 'bold'), fg=accent, bg=bg).pack(side=tk.LEFT)
        self.status_label = tk.Label(title_frame, text="● 连接中...",
                                     font=('Helvetica', 11), fg=yellow, bg=bg)
        self.status_label.pack(side=tk.RIGHT)

        # 主内容区域
        main_frame = tk.Frame(self.root, bg=bg)
        main_frame.pack(fill=tk.BOTH, expand=True, padx=15, pady=5)

        # 左侧 - 信息面板
        left_frame = tk.Frame(main_frame, bg=bg, width=340)
        left_frame.pack(side=tk.LEFT, fill=tk.Y, padx=(0, 10))
        left_frame.pack_propagate(False)

        self.labels = {}

        # --- 当前坐标面板 ---
        coord_frame = tk.LabelFrame(left_frame, text=" 当前坐标 ", font=('Helvetica', 12, 'bold'),
                                     fg=accent, bg='#313244', bd=2, relief=tk.GROOVE)
        coord_frame.pack(fill=tk.X, pady=(0, 6))

        coord_fields = [
            ('x', 'X 坐标', '0.000 m', green),
            ('y', 'Y 坐标', '0.000 m', green),
            ('z', 'Z 坐标', '0.000 m', green),
        ]
        for key, label_text, default, color in coord_fields:
            row = tk.Frame(coord_frame, bg='#313244')
            row.pack(fill=tk.X, padx=10, pady=2)
            tk.Label(row, text=label_text, font=('Helvetica', 10),
                     fg='#a6adc8', bg='#313244', width=14, anchor='w').pack(side=tk.LEFT)
            lbl = tk.Label(row, text=default, font=('Consolas', 12, 'bold'),
                           fg=color, bg='#313244', anchor='e')
            lbl.pack(side=tk.RIGHT, fill=tk.X, expand=True)
            self.labels[key] = lbl

        # --- 位置变化量面板 (新增) ---
        delta_frame = tk.LabelFrame(left_frame, text=" 位置变化量 (相对上一时刻) ", font=('Helvetica', 12, 'bold'),
                                     fg='#fab387', bg='#313244', bd=2, relief=tk.GROOVE)
        delta_frame.pack(fill=tk.X, pady=(0, 6))

        delta_fields = [
            ('dx', 'ΔX 变化', '+0.000 m', '#f38ba8'),
            ('dy', 'ΔY 变化', '+0.000 m', '#89b4fa'),
            ('dz', 'ΔZ 变化', '+0.000 m', '#a6e3a1'),
            ('d_total', '总位移量', '0.000 m', '#fab387'),
        ]
        for key, label_text, default, color in delta_fields:
            row = tk.Frame(delta_frame, bg='#313244')
            row.pack(fill=tk.X, padx=10, pady=2)
            tk.Label(row, text=label_text, font=('Helvetica', 10),
                     fg='#a6adc8', bg='#313244', width=14, anchor='w').pack(side=tk.LEFT)
            lbl = tk.Label(row, text=default, font=('Consolas', 12, 'bold'),
                           fg=color, bg='#313244', anchor='e')
            lbl.pack(side=tk.RIGHT, fill=tk.X, expand=True)
            self.labels[key] = lbl

        # --- 朝向角度面板 ---
        angle_frame = tk.LabelFrame(left_frame, text=" 朝向角度 ", font=('Helvetica', 12, 'bold'),
                                     fg=accent, bg='#313244', bd=2, relief=tk.GROOVE)
        angle_frame.pack(fill=tk.X, pady=(0, 6))

        angle_fields = [
            ('yaw', '偏航角 Yaw', '0.0°', accent),
            ('pitch', '俯仰角 Pitch', '0.0°', accent),
            ('roll', '横滚角 Roll', '0.0°', accent),
        ]
        for key, label_text, default, color in angle_fields:
            row = tk.Frame(angle_frame, bg='#313244')
            row.pack(fill=tk.X, padx=10, pady=2)
            tk.Label(row, text=label_text, font=('Helvetica', 10),
                     fg='#a6adc8', bg='#313244', width=14, anchor='w').pack(side=tk.LEFT)
            lbl = tk.Label(row, text=default, font=('Consolas', 11, 'bold'),
                           fg=color, bg='#313244', anchor='e')
            lbl.pack(side=tk.RIGHT, fill=tk.X, expand=True)
            self.labels[key] = lbl

        # --- 状态信息面板 ---
        status_frame = tk.LabelFrame(left_frame, text=" 状态信息 ", font=('Helvetica', 12, 'bold'),
                                      fg=accent, bg='#313244', bd=2, relief=tk.GROOVE)
        status_frame.pack(fill=tk.X, pady=(0, 6))

        status_fields = [
            ('distance', '距原点距离', '0.000 m', yellow),
            ('speed', '移动速度', '0.000 m/s', yellow),
            ('position', '位置范围', '正常', green),
        ]
        for key, label_text, default, color in status_fields:
            row = tk.Frame(status_frame, bg='#313244')
            row.pack(fill=tk.X, padx=10, pady=2)
            tk.Label(row, text=label_text, font=('Helvetica', 10),
                     fg='#a6adc8', bg='#313244', width=14, anchor='w').pack(side=tk.LEFT)
            lbl = tk.Label(row, text=default, font=('Consolas', 11, 'bold'),
                           fg=color, bg='#313244', anchor='e')
            lbl.pack(side=tk.RIGHT, fill=tk.X, expand=True)
            self.labels[key] = lbl

        # 右侧 - 轨迹地图
        right_frame = tk.Frame(main_frame, bg='#313244', bd=2, relief=tk.GROOVE)
        right_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)

        map_title = tk.Frame(right_frame, bg='#313244')
        map_title.pack(fill=tk.X, padx=5, pady=5)
        tk.Label(map_title, text="运动轨迹地图", font=('Helvetica', 12, 'bold'),
                 fg=accent, bg='#313244').pack(side=tk.LEFT)
        self.map_info_label = tk.Label(map_title, text="", font=('Consolas', 9),
                                        fg='#a6adc8', bg='#313244')
        self.map_info_label.pack(side=tk.RIGHT)

        self.canvas = tk.Canvas(right_frame, bg='#181825', highlightthickness=0)
        self.canvas.pack(fill=tk.BOTH, expand=True, padx=5, pady=(0, 5))

    def update_display(self, x, y, z, roll, pitch, yaw, distance, speed, dx, dy, dz, trajectory, 
                      map_origin_x=0.0, map_origin_y=0.0, map_origin_z=0.0):
        # 当前坐标
        self.labels['x'].config(text=f"{x:+.3f} m")
        self.labels['y'].config(text=f"{y:+.3f} m")
        self.labels['z'].config(text=f"{z:+.3f} m")

        # 位置变化量
        self.labels['dx'].config(text=f"{dx:+.4f} m")
        self.labels['dy'].config(text=f"{dy:+.4f} m")
        self.labels['dz'].config(text=f"{dz:+.4f} m")
        d_total = math.sqrt(dx*dx + dy*dy + dz*dz)
        self.labels['d_total'].config(text=f"{d_total:.4f} m")

        # 变化量颜色：有变化时高亮
        for key, val in [('dx', dx), ('dy', dy), ('dz', dz)]:
            if abs(val) > 0.01:
                self.labels[key].config(fg='#f9e2af')
            else:
                self.labels[key].config(fg='#585b70')

        if d_total > 0.05:
            self.labels['d_total'].config(fg='#f38ba8')
        elif d_total > 0.01:
            self.labels['d_total'].config(fg='#f9e2af')
        else:
            self.labels['d_total'].config(fg='#585b70')

        # 朝向角度
        self.labels['yaw'].config(text=f"{math.degrees(yaw):+.1f}°")
        self.labels['pitch'].config(text=f"{math.degrees(pitch):+.1f}°")
        self.labels['roll'].config(text=f"{math.degrees(roll):+.1f}°")

        # 状态
        self.labels['distance'].config(text=f"{distance:.3f} m")
        self.labels['speed'].config(text=f"{speed:.3f} m/s")
        self.status_label.config(text="● 已连接", fg='#a6e3a1')

        origin_dist = math.sqrt(x * x + y * y)
        if origin_dist > 30:
            self.labels['position'].config(text="远离原点!", fg='#f38ba8')
        elif origin_dist > 15:
            self.labels['position'].config(text="较远", fg='#f9e2af')
        else:
            self.labels['position'].config(text="正常", fg='#a6e3a1')

        self._draw_trajectory(trajectory, x, y, map_origin_x, map_origin_y)

    def _draw_trajectory(self, trajectory, cur_x, cur_y, map_origin_x=0.0, map_origin_y=0.0):
        self.canvas.delete("all")
        w = self.canvas.winfo_width()
        h = self.canvas.winfo_height()
        if w < 50 or h < 50:
            return

        if len(trajectory) < 2:
            self.canvas.create_text(w // 2, h // 2, text="等待轨迹数据...",
                                    fill='#585b70', font=('Helvetica', 14))
            return

        xs = [p[0] for p in trajectory]
        ys = [p[1] for p in trajectory]

        margin = 40
        x_min, x_max = min(xs), max(xs)
        y_min, y_max = min(ys), max(ys)

        x_range = x_max - x_min
        y_range = y_max - y_min
        if x_range < 0.5:
            x_range = 0.5
        if y_range < 0.5:
            y_range = 0.5

        scale_x = (w - 2 * margin) / x_range
        scale_y = (h - 2 * margin) / y_range
        scale = min(scale_x, scale_y)

        cx = (x_min + x_max) / 2
        cy = (y_min + y_max) / 2

        def to_screen(px, py):
            sx = w / 2 + (px - cx) * scale
            sy = h / 2 - (py - cy) * scale
            return sx, sy

        # 绘制网格
        grid_step = self._calc_grid_step(x_range, y_range)
        gx = math.floor(x_min / grid_step) * grid_step
        while gx <= x_max:
            sx, _ = to_screen(gx, 0)
            if margin < sx < w - margin:
                self.canvas.create_line(sx, margin, sx, h - margin, fill='#313244', width=1)
                self.canvas.create_text(sx, h - margin + 10, text=f"{gx:.0f}",
                                        fill='#585b70', font=('Helvetica', 7))
            gx += grid_step
        gy = math.floor(y_min / grid_step) * grid_step
        while gy <= y_max:
            _, sy = to_screen(0, gy)
            if margin < sy < h - margin:
                self.canvas.create_line(margin, sy, w - margin, sy, fill='#313244', width=1)
                self.canvas.create_text(margin - 15, sy, text=f"{gy:.0f}",
                                        fill='#585b70', font=('Helvetica', 7))
            gy += grid_step

        # 绘制坐标轴
        ox, oy = to_screen(0, 0)
        if margin < ox < w - margin:
            self.canvas.create_line(ox, margin, ox, h - margin, fill='#45475a', width=1, dash=(4, 4))
        if margin < oy < h - margin:
            self.canvas.create_line(margin, oy, w - margin, oy, fill='#45475a', width=1, dash=(4, 4))

        # 绘制轨迹
        points = []
        for px, py in trajectory:
            sx, sy = to_screen(px, py)
            points.append((sx, sy))

        for i in range(1, len(points)):
            alpha = i / len(points)
            r = int(0x30 + 0x59 * alpha)
            g = int(0x30 + 0xb3 * alpha)
            b = int(0xf0 + 0x0b * alpha)
            color = f'#{r:02x}{g:02x}{b:02x}'
            self.canvas.create_line(points[i-1][0], points[i-1][1],
                                    points[i][0], points[i][1],
                                    fill=color, width=2.5, capstyle=tk.ROUND)

        # 绘制起点
        if points:
            sx, sy = points[0]
            self.canvas.create_oval(sx - 5, sy - 5, sx + 5, sy + 5,
                                    fill='#f38ba8', outline='#f38ba8')
            self.canvas.create_text(sx + 12, sy, text="起", fill='#f38ba8',
                                    font=('Helvetica', 8), anchor='w')

        # 绘制当前位置
        if points:
            sx, sy = points[-1]
            self.canvas.create_oval(sx - 7, sy - 7, sx + 7, sy + 7,
                                    fill='#a6e3a1', outline='#1e1e2e', width=2)
            self.canvas.create_text(sx + 12, sy, text="当前位置", fill='#a6e3a1',
                                    font=('Helvetica', 9, 'bold'), anchor='w')

        # 原点标记 (camera_init坐标系原点)
        ox_s, oy_s = to_screen(0, 0)
        if margin < ox_s < w - margin and margin < oy_s < h - margin:
            self.canvas.create_line(ox_s - 8, oy_s, ox_s + 8, oy_s, fill='#f9e2af', width=2)
            self.canvas.create_line(ox_s, oy_s - 8, ox_s, oy_s + 8, fill='#f9e2af', width=2)
            self.canvas.create_text(ox_s + 10, oy_s - 10, text="O", fill='#f9e2af',
                                    font=('Helvetica', 9, 'bold'), anchor='w')
        
        # 建图原点标记 (地图文件原点)
        map_ox_s, map_oy_s = to_screen(map_origin_x, map_origin_y)
        if margin < map_ox_s < w - margin and margin < map_oy_s < h - margin:
            # 绘制红色十字标记
            self.canvas.create_line(map_ox_s - 10, map_oy_s, map_ox_s + 10, map_oy_s, fill='#f38ba8', width=3)
            self.canvas.create_line(map_ox_s, map_oy_s - 10, map_ox_s, map_oy_s + 10, fill='#f38ba8', width=3)
            # 绘制红色圆圈
            self.canvas.create_oval(map_ox_s - 12, map_oy_s - 12, map_ox_s + 12, map_oy_s + 12, 
                                   outline='#f38ba8', width=2)
            # 添加文字标签
            self.canvas.create_text(map_ox_s + 15, map_oy_s - 15, text="建图原点", fill='#f38ba8',
                                    font=('Helvetica', 9, 'bold'), anchor='w')
            self.canvas.create_text(map_ox_s + 15, map_oy_s + 5, 
                                   text=f"({map_origin_x:.1f}, {map_origin_y:.1f})", 
                                   fill='#f38ba8', font=('Helvetica', 8), anchor='w')

        self.map_info_label.config(
            text=f"点数:{len(trajectory)}  范围:{x_range:.1f}x{y_range:.1f}m  缩放:{scale:.1f}")

    def _calc_grid_step(self, x_range, y_range):
        max_range = max(x_range, y_range)
        if max_range < 2:
            return 0.5
        elif max_range < 5:
            return 1
        elif max_range < 20:
            return 2
        elif max_range < 50:
            return 5
        elif max_range < 100:
            return 10
        else:
            return 20

    def run(self):
        self.root.mainloop()


def main(args=None):
    rclpy.init(args=args)

    gui = None
    node = None

    def gui_callback(x, y, z, roll, pitch, yaw, distance, speed, dx, dy, dz, trajectory, 
                    map_origin_x=0.0, map_origin_y=0.0, map_origin_z=0.0):
        if gui:
            gui.root.after(0, lambda: gui.update_display(
                x, y, z, roll, pitch, yaw, distance, speed, dx, dy, dz, trajectory,
                map_origin_x, map_origin_y, map_origin_z))

    gui = CoordinateDisplayGUI()
    node = CoordinateDisplayNode(gui_callback)

    ros_thread = threading.Thread(target=lambda: rclpy.spin(node), daemon=True)
    ros_thread.start()

    def on_closing():
        node.destroy_node()
        rclpy.shutdown()
        gui.root.destroy()

    gui.root.protocol("WM_DELETE_WINDOW", on_closing)
    gui.run()


if __name__ == '__main__':
    main()
