import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.animation import FuncAnimation
import threading
import time

class SimpleElevatorGUI(Node):

    def __init__(self):
        super().__init__('simple_elevator_gui')

        # Subscribe to scan and goal
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        self.goal_sub = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_callback,
            10
        )

        # Store latest data
        self.latest_scan = None
        self.latest_goal = None

        # GUI parameters
        self.grid_resolution = 0.05  # Same as in free_space_goal.py
        self.grid_size = 100
        self.robot_radius = 0.3

        # Start GUI in separate thread
        self.gui_thread = threading.Thread(target=self.run_gui)
        self.gui_thread.daemon = True
        self.gui_thread.start()

        # Timer to update GUI
        self.timer = self.create_timer(0.5, self.update_gui_data)

    def scan_callback(self, msg):
        self.latest_scan = msg

    def goal_callback(self, msg):
        self.latest_goal = msg

    def update_gui_data(self):
        # This just triggers the GUI update in the matplotlib thread
        pass

    def run_gui(self):
        """Run matplotlib GUI in separate thread"""
        self.fig, self.ax = plt.subplots(figsize=(8, 8))
        self.setup_plot()

        # Animation to update plot
        ani = FuncAnimation(self.fig, self.update_plot, interval=500, cache_frame_data=False)

        plt.show()

    def setup_plot(self):
        """Setup the plot appearance"""
        self.ax.set_title('Elevator Navigation - Free Space & Goal', fontsize=14, fontweight='bold')
        self.ax.set_xlabel('X (meters)')
        self.ax.set_ylabel('Y (meters)')
        self.ax.grid(True, alpha=0.3)
        self.ax.set_aspect('equal')

        # Set axis limits (5m x 5m around robot)
        self.ax.set_xlim(-2.5, 2.5)
        self.ax.set_ylim(-2.5, 2.5)

        # Add robot position marker
        self.robot_marker, = self.ax.plot(0, 0, 'ko', markersize=10, label='Robot')

        # Initialize empty collections for dynamic elements
        self.scan_points = self.ax.scatter([], [], c='red', s=2, alpha=0.6, label='Laser Points')
        self.free_patches = []  # List to store free region patches
        self.goal_marker, = self.ax.plot([], [], 'go', markersize=15, alpha=0.8, label='Goal')

        self.ax.legend(loc='upper right')

    def update_plot(self, frame):
        """Update the plot with latest data"""
        if self.latest_scan is None:
            return

        # Clear previous free region patches
        for patch in self.free_patches:
            patch.remove()
        self.free_patches = []

        # 1. Update laser scan points
        self.update_scan_points()

        # 2. Update free regions
        self.update_free_regions()

        # 3. Update goal position
        self.update_goal_position()

        # Update title with status
        num_regions = len(self.free_patches)
        status = f"Elevator Navigation - Free Space & Goal (Regions: {num_regions})"
        if self.latest_goal:
            status += " ✓"
        self.ax.set_title(status, fontsize=14, fontweight='bold')

    def update_scan_points(self):
        """Update laser scan points visualization"""
        if self.latest_scan is None:
            return

        # Convert scan to points (same as in free_space_goal.py)
        ranges = np.array(self.latest_scan.ranges)
        angles = np.arange(len(ranges)) * self.latest_scan.angle_increment + self.latest_scan.angle_min

        points_x = []
        points_y = []

        for angle, r in zip(angles, ranges):
            if r < self.latest_scan.range_min or r > self.latest_scan.range_max:
                continue

            # ROI filter (front 90 degrees)
            if abs(angle) > np.pi / 2:
                continue

            # Distance filter
            if r > 5.0:
                continue

            x = r * np.cos(angle)
            y = r * np.sin(angle)
            points_x.append(x)
            points_y.append(y)

        # Update scatter plot
        self.scan_points.set_offsets(np.column_stack([points_x, points_y]))

    def update_free_regions(self):
        """Update free regions visualization"""
        if self.latest_scan is None:
            return

        # Get points and build grid (same as in free_space_goal.py)
        points = self.scan_to_points(self.latest_scan)
        if len(points) == 0:
            return

        grid = self.build_grid(points)
        grid = self.inflate_grid(grid)
        regions = self.find_free_regions(grid)

        # Colors for different regions
        colors = ['lightgreen', 'lightblue', 'lightyellow', 'lightpink', 'lightcyan']

        # Create patches for each region
        center = self.grid_size // 2
        for i, region in enumerate(regions):
            if len(region) < 10:  # Skip very small regions
                continue

            # Convert region cells to world coordinates
            region_points = []
            for y, x in region:
                world_x = (x - center) * self.grid_resolution
                world_y = (y - center) * self.grid_resolution
                region_points.append([world_x, world_y])

            if len(region_points) > 2:
                region_array = np.array(region_points)
                # Create convex hull approximation
                try:
                    from scipy.spatial import ConvexHull
                    hull = ConvexHull(region_array)
                    hull_points = region_array[hull.vertices]

                    # Create polygon patch
                    color = colors[i % len(colors)]
                    patch = patches.Polygon(hull_points, alpha=0.5, color=color,
                                          edgecolor='darkgreen', linewidth=1)
                    self.ax.add_patch(patch)
                    self.free_patches.append(patch)
                except:
                    # Fallback: just plot points
                    pass

    def update_goal_position(self):
        """Update goal position marker"""
        if self.latest_goal is None:
            self.goal_marker.set_data([], [])
            return

        x = self.latest_goal.pose.position.x
        y = self.latest_goal.pose.position.y

        self.goal_marker.set_data([x], [y])

    # Helper methods (same as in free_space_goal.py)
    def scan_to_points(self, msg):
        roi_angle = np.pi / 2
        max_range = 5.0

        ranges = np.array(msg.ranges)
        angles = np.arange(len(ranges)) * msg.angle_increment + msg.angle_min

        points = []
        for angle, r in zip(angles, ranges):
            if abs(angle) > roi_angle:
                continue
            if r < msg.range_min or r > msg.range_max or r > max_range:
                continue

            x = r * np.cos(angle)
            y = r * np.sin(angle)
            points.append((x, y))

        return np.array(points) if points else np.array([])

    def world_to_grid(self, x, y):
        center = self.grid_size // 2
        gx = int(center + x / self.grid_resolution)
        gy = int(center + y / self.grid_resolution)
        return gx, gy

    def raytrace_cells(self, x0, y0, x1, y1):
        cells = []
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy
        x, y = x0, y0

        while True:
            cells.append((y, x))
            if x == x1 and y == y1:
                break
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x += sx
            if e2 < dx:
                err += dx
                y += sy

        return cells

    def build_grid(self, points):
        grid = np.zeros((self.grid_size, self.grid_size), dtype=np.uint8)
        center = self.grid_size // 2

        for x, y in points:
            gx, gy = self.world_to_grid(x, y)
            if not (0 <= gx < self.grid_size and 0 <= gy < self.grid_size):
                continue

            line = self.raytrace_cells(center, center, gx, gy)
            for cy, cx in line[:-1]:
                if 0 <= cx < self.grid_size and 0 <= cy < self.grid_size:
                    grid[cy, cx] = 0
            grid[gy, gx] = 100

        return grid

    def inflate_grid(self, grid):
        from scipy import ndimage
        inflate_cells = int(np.ceil(self.robot_radius / self.grid_resolution))
        kernel = ndimage.generate_binary_structure(2, 2)
        inflated = ndimage.binary_dilation(grid > 0, structure=kernel, iterations=inflate_cells)
        return (inflated * 100).astype(np.uint8)

    def find_free_regions(self, grid):
        free_mask = (grid == 0).astype(np.uint8)
        visited = np.zeros_like(free_mask)
        regions = []
        min_area = 10

        for i in range(grid.shape[0]):
            for j in range(grid.shape[1]):
                if free_mask[i, j] == 1 and visited[i, j] == 0:
                    stack = [(i, j)]
                    region = []

                    while stack:
                        y, x = stack.pop()
                        if x < 0 or x >= grid.shape[1] or y < 0 or y >= grid.shape[0]:
                            continue
                        if visited[y, x] or free_mask[y, x] == 0:
                            continue

                        visited[y, x] = 1
                        region.append((y, x))

                        for dy, dx in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                            stack.append((y + dy, x + dx))

                    if len(region) > min_area:
                        regions.append(region)

        return regions


def main():
    rclpy.init()
    node = SimpleElevatorGUI()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()