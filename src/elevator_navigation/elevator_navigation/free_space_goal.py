import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped
import numpy as np
from scipy import ndimage

class ScanToGoal(Node):

    def __init__(self):
        super().__init__('scan_to_goal')

        self.sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        self.pub = self.create_publisher(
            PoseStamped,
            '/goal_pose',
            10
        )

        # robot params
        self.robot_radius = 0.3  # m
        self.grid_resolution = 0.05  # m/cell (20cm × 20cm)
        self.grid_size = 100  # 5m × 5m (100 cells × 5m / 0.05)
        
        # ROI params
        self.roi_angle = np.pi / 2  # 90 degrees, focus on front
        self.max_range = 5.0  # m, max lidar range for cabin
        self.min_area = 10  # min cells for valid region

    def scan_callback(self, msg):
        """Main pipeline: scan → points → grid → inflate → regions → goal"""
        
        # Step 1: Convert scan to points with ROI filter
        points = self.scan_to_points(msg)
        
        if len(points) == 0:
            self.get_logger().warn("No points in ROI")
            return
        
        # Step 2: Build local occupancy grid
        grid = self.build_grid(points)
        
        # Step 3: Inflate grid by robot radius
        grid = self.inflate_grid(grid)
        
        # Step 4: Find free space regions
        regions = self.find_free_regions(grid)
        
        if not regions:
            self.get_logger().warn("No free regions found")
            return
        
        # Step 5: Select best region (largest in front)
        best_region = self.select_best_region(regions)
        
        if best_region is None:
            self.get_logger().warn("No valid region selected")
            return
        
        # Step 6: Calculate centroid
        centroid_grid = self.calculate_centroid(best_region)
        
        # Step 7: Convert grid coords to world coords
        goal_pose = self.grid_to_world(centroid_grid, msg.header.frame_id)
        
        self.pub.publish(goal_pose)

    def scan_to_points(self, msg):
        """Convert LaserScan to (x,y) points with ROI and distance filter"""
        ranges = np.array(msg.ranges)
        angles = np.arange(len(ranges)) * msg.angle_increment + msg.angle_min
        
        points = []
        for angle, r in zip(angles, ranges):
            # Filter 1: ROI (front 90 degrees)
            if abs(angle) > self.roi_angle:
                continue
            
            # Filter 2: Valid range
            if r < msg.range_min or r > msg.range_max or r > self.max_range:
                continue
            
            # Convert to x,y
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
        """Bresenham line between two grid cells."""
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
        """Build occupancy grid from points by raycasting from robot."""
        grid = np.zeros((self.grid_size, self.grid_size), dtype=np.uint8)
        center = self.grid_size // 2

        for x, y in points:
            gx, gy = self.world_to_grid(x, y)
            if not (0 <= gx < self.grid_size and 0 <= gy < self.grid_size):
                continue

            line = self.raytrace_cells(center, center, gx, gy)
            # Mark free cells first
            for cy, cx in line[:-1]:
                if 0 <= cx < self.grid_size and 0 <= cy < self.grid_size:
                    grid[cy, cx] = 0
            # Mark endpoint as occupied
            grid[gy, gx] = 100

        return grid

    def inflate_grid(self, grid):
        """Inflate obstacles by robot radius"""
        inflate_cells = int(np.ceil(self.robot_radius / self.grid_resolution))
        
        # Use binary dilation to expand occupied cells
        kernel = ndimage.generate_binary_structure(2, 2)
        inflated = ndimage.binary_dilation(grid > 0, structure=kernel, iterations=inflate_cells)
        
        return (inflated * 100).astype(np.uint8)

    def find_free_regions(self, grid):
        """Find connected free space regions (value=0)"""
        free_mask = (grid == 0).astype(np.uint8)
        visited = np.zeros_like(free_mask)
        regions = []
        
        # Flood fill to find connected components
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
                        
                        # 4-connectivity
                        for dy, dx in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                            stack.append((y + dy, x + dx))
                    
                    if len(region) > self.min_area:
                        regions.append(region)
        
        return regions

    def select_best_region(self, regions):
        """Select largest region in front of robot"""
        if not regions:
            return None
        
        center = self.grid_size // 2
        best_region = None
        best_score = -1
        
        for region in regions:
            region_array = np.array(region)
            avg_x = np.mean(region_array[:, 1])  # col index = x
            avg_y = np.mean(region_array[:, 0])  # row index = y

            # Only consider regions in front half of map
            if avg_x <= center:
                continue

            area = len(region)
            front_bonus = max(0, (avg_x - center) * 10)
            y_center_ratio = 1.0 - abs(avg_y - center) / center
            center_align_bonus = max(0, y_center_ratio) * 50
            score = area + front_bonus + center_align_bonus

            if score > best_score:
                best_score = score
                best_region = region
        
        return best_region

    def calculate_centroid(self, region):
        """Calculate centroid of region (grid coords)"""
        region_array = np.array(region)
        centroid_y = int(np.mean(region_array[:, 0]))
        centroid_x = int(np.mean(region_array[:, 1]))
        return (centroid_x, centroid_y)

    def grid_to_world(self, grid_coords, frame_id):
        """Convert grid coordinates to world coordinates"""
        grid_x, grid_y = grid_coords
        center = self.grid_size // 2
        
        # Convert grid to world (center of grid is robot at 0,0)
        world_x = (grid_x - center) * self.grid_resolution
        world_y = (grid_y - center) * self.grid_resolution
        
        goal = PoseStamped()
        goal.header.frame_id = frame_id
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = world_x
        goal.pose.position.y = world_y
        goal.pose.orientation.w = 1.0
        
        return goal


def main():
    rclpy.init()
    node = ScanToGoal()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()