import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np
import math

class ElevatorVisualizer(Node):

    def __init__(self):
        super().__init__('elevator_visualizer')

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

        # Publishers for visualization
        self.marker_pub = self.create_publisher(
            MarkerArray,
            '/elevator_visualization',
            10
        )

        # Store latest data
        self.latest_scan = None
        self.latest_goal = None

        # Visualization params
        self.grid_resolution = 0.05  # Same as in free_space_goal.py
        self.grid_size = 100
        self.robot_radius = 0.3

        # Timer to update visualization
        self.timer = self.create_timer(0.5, self.update_visualization)

    def scan_callback(self, msg):
        self.latest_scan = msg

    def goal_callback(self, msg):
        self.latest_goal = msg

    def update_visualization(self):
        if self.latest_scan is None:
            return

        marker_array = MarkerArray()

        # 1. Visualize laser scan points
        scan_marker = self.create_scan_marker(self.latest_scan)
        marker_array.markers.append(scan_marker)

        # 2. Visualize occupancy grid and free regions
        if self.latest_scan is not None:
            grid_markers = self.create_grid_markers(self.latest_scan)
            marker_array.markers.extend(grid_markers)

        # 3. Visualize goal pose
        if self.latest_goal is not None:
            goal_marker = self.create_goal_marker(self.latest_goal)
            marker_array.markers.append(goal_marker)

        # Clear old markers
        for i, marker in enumerate(marker_array.markers):
            marker.id = i
            marker.header.frame_id = self.latest_scan.header.frame_id
            marker.header.stamp = self.get_clock().now().to_msg()

        self.marker_pub.publish(marker_array)

    def create_scan_marker(self, scan_msg):
        """Create marker for laser scan points"""
        marker = Marker()
        marker.type = Marker.POINTS
        marker.action = Marker.ADD
        marker.scale.x = 0.02
        marker.scale.y = 0.02
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 0.8

        # Convert scan to points
        ranges = np.array(scan_msg.ranges)
        angles = np.arange(len(ranges)) * scan_msg.angle_increment + scan_msg.angle_min

        for angle, r in zip(angles, ranges):
            if r < scan_msg.range_min or r > scan_msg.range_max:
                continue

            x = r * np.cos(angle)
            y = r * np.sin(angle)

            # Add point
            marker.points.append(Point(x=x, y=y, z=0.0))

        return marker

    def create_grid_markers(self, scan_msg):
        """Create markers for occupancy grid and free regions"""
        markers = []

        # Convert scan to points (same as in free_space_goal.py)
        points = self.scan_to_points(scan_msg)
        if len(points) == 0:
            return markers

        # Build grid
        grid = self.build_grid(points)

        # Inflate grid
        grid = self.inflate_grid(grid)

        # Find free regions
        regions = self.find_free_regions(grid)

        # Create marker for occupied cells (red cubes)
        occupied_marker = Marker()
        occupied_marker.type = Marker.CUBE_LIST
        occupied_marker.action = Marker.ADD
        occupied_marker.scale.x = self.grid_resolution
        occupied_marker.scale.y = self.grid_resolution
        occupied_marker.scale.z = 0.01
        occupied_marker.color.r = 1.0
        occupied_marker.color.g = 0.0
        occupied_marker.color.b = 0.0
        occupied_marker.color.a = 0.5

        # Create marker for inflated obstacles (orange cubes)
        inflated_marker = Marker()
        inflated_marker.type = Marker.CUBE_LIST
        inflated_marker.action = Marker.ADD
        inflated_marker.scale.x = self.grid_resolution
        inflated_marker.scale.y = self.grid_resolution
        inflated_marker.scale.z = 0.01
        inflated_marker.color.r = 1.0
        inflated_marker.color.g = 0.5
        inflated_marker.color.b = 0.0
        inflated_marker.color.a = 0.3

        # Create markers for free regions (green cubes)
        free_markers = []
        colors = [
            (0.0, 1.0, 0.0, 0.3),  # Green
            (0.0, 0.0, 1.0, 0.3),  # Blue
            (1.0, 0.0, 1.0, 0.3),  # Magenta
            (0.0, 1.0, 1.0, 0.3),  # Cyan
        ]

        center = self.grid_size // 2

        for i in range(self.grid_size):
            for j in range(self.grid_size):
                world_x = (j - center) * self.grid_resolution
                world_y = (i - center) * self.grid_resolution

                if grid[i, j] == 100:  # Occupied
                    occupied_marker.points.append(Point(x=world_x, y=world_y, z=0.0))
                elif grid[i, j] == 0:  # Free
                    # Find which region this cell belongs to
                    region_idx = -1
                    for idx, region in enumerate(regions):
                        if (i, j) in region:
                            region_idx = idx
                            break

                    if region_idx >= 0:
                        # Create marker for this region if not exists
                        while len(free_markers) <= region_idx:
                            marker = Marker()
                            marker.type = Marker.CUBE_LIST
                            marker.action = Marker.ADD
                            marker.scale.x = self.grid_resolution
                            marker.scale.y = self.grid_resolution
                            marker.scale.z = 0.01
                            r, g, b, a = colors[len(free_markers) % len(colors)]
                            marker.color.r = r
                            marker.color.g = g
                            marker.color.b = b
                            marker.color.a = a
                            free_markers.append(marker)

                        free_markers[region_idx].points.append(Point(x=world_x, y=world_y, z=0.0))

        markers.append(occupied_marker)
        markers.extend(free_markers)

        return markers

    def create_goal_marker(self, goal_msg):
        """Create marker for goal pose"""
        marker = Marker()
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose = goal_msg.pose
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        return marker

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

    def build_grid(self, points):
        grid = np.zeros((self.grid_size, self.grid_size), dtype=np.uint8)
        center = self.grid_size // 2

        for x, y in points:
            grid_x = int(center + x / self.grid_resolution)
            grid_y = int(center + y / self.grid_resolution)

            if 0 <= grid_x < self.grid_size and 0 <= grid_y < self.grid_size:
                grid[grid_y, grid_x] = 100

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
    node = ElevatorVisualizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()