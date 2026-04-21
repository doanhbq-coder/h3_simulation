#!/usr/bin/env python3
"""
Demo script để test GUI đơn giản mà không cần ROS2
Tạo fake laser scan data và goal pose để test visualization
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.animation import FuncAnimation
import time

class DemoGUI:
    def __init__(self):
        # Fake elevator environment
        self.create_fake_elevator_data()

        # Setup plot
        self.fig, self.ax = plt.subplots(figsize=(8, 8))
        self.setup_plot()

        # Animation
        ani = FuncAnimation(self.fig, self.update_plot, interval=1000, cache_frame_data=False)
        plt.show()

    def create_fake_elevator_data(self):
        """Tạo fake data mô phỏng thang máy"""
        # Thang máy hình chữ nhật 2m x 1.5m
        elevator_width = 2.0
        elevator_height = 1.5
        elevator_distance = 1.5  # Khoảng cách từ robot đến thang máy

        # Tạo laser scan points (mô phỏng LIDAR)
        angles = np.linspace(-np.pi/2, np.pi/2, 360)  # 180 degrees front
        ranges = np.full_like(angles, 5.0)  # Max range

        # Thêm obstacles (tường thang máy)
        # Tạo vùng thang máy
        front_angles = (angles >= -np.pi/3) & (angles <= np.pi/3)  # 120 degrees front
        side_angles = (angles < -np.pi/3) | (angles > np.pi/3)     # sides

        # Tường trước (front wall)
        ranges[front_angles] = elevator_distance

        # Tường bên (side walls) - chỉ ở góc nhỏ
        left_mask = (angles >= -np.pi/2) & (angles <= -np.pi/4)
        right_mask = (angles >= np.pi/4) & (angles <= np.pi/2)

        # Tạo hiệu ứng tường bên
        for i, angle in enumerate(angles):
            if left_mask[i]:
                # Tường trái
                wall_distance = elevator_distance - elevator_width/2
                ranges[i] = wall_distance
            elif right_mask[i]:
                # Tường phải
                wall_distance = elevator_distance + elevator_width/2
                ranges[i] = wall_distance

        # Thêm noise
        ranges += np.random.normal(0, 0.02, len(ranges))
        ranges = np.clip(ranges, 0.1, 5.0)

        self.scan_angles = angles
        self.scan_ranges = ranges

        # Fake goal pose (trung tâm thang máy)
        self.goal_x = elevator_distance
        self.goal_y = 0.0

    def setup_plot(self):
        """Setup plot"""
        self.ax.set_title('Elevator Navigation Demo - Free Space & Goal', fontsize=14, fontweight='bold')
        self.ax.set_xlabel('X (meters)')
        self.ax.set_ylabel('Y (meters)')
        self.ax.grid(True, alpha=0.3)
        self.ax.set_aspect('equal')
        self.ax.set_xlim(-1, 3)
        self.ax.set_ylim(-2, 2)

        # Robot position
        self.robot_marker, = self.ax.plot(0, 0, 'ko', markersize=10, label='Robot')

        # Scan points
        self.scan_points = self.ax.scatter([], [], c='red', s=2, alpha=0.6, label='Laser Points')

        # Free regions
        self.free_patches = []

        # Goal
        self.goal_marker, = self.ax.plot([self.goal_x], [self.goal_y], 'go', markersize=15, alpha=0.8, label='Goal')

        self.ax.legend(loc='upper right')

    def update_plot(self, frame):
        """Update plot"""
        # Clear previous patches
        for patch in self.free_patches:
            patch.remove()
        self.free_patches = []

        # Update scan points
        x_points = self.scan_ranges * np.cos(self.scan_angles)
        y_points = self.scan_ranges * np.sin(self.scan_angles)
        self.scan_points.set_offsets(np.column_stack([x_points, y_points]))

        # Simulate free regions (thang máy)
        # Vùng free space trong thang máy
        elevator_rect = patches.Rectangle((0.5, -0.75), 2.0, 1.5,
                                       alpha=0.5, color='lightgreen',
                                       edgecolor='darkgreen', linewidth=2)
        self.ax.add_patch(elevator_rect)
        self.free_patches.append(elevator_rect)

        # Update title
        self.ax.set_title(f'Elevator Navigation Demo - Free Space & Goal (Frame: {frame})',
                         fontsize=14, fontweight='bold')

if __name__ == '__main__':
    print("Starting Elevator Navigation Demo GUI...")
    print("Close the window to exit")
    demo = DemoGUI()