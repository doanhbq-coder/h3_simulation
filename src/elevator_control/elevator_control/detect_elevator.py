import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PointStamped
import math

class ElevatorDetector(Node):

    def __init__(self):
        super().__init__('elevator_detector')

        self.sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        self.pub = self.create_publisher(
            PointStamped,
            '/elevator_center',
            10
        )

        self.gap_threshold = 2.5  # m
        self.min_gap_width = 1.2  # m

    def scan_callback(self, msg):
        ranges = msg.ranges
        angle_min = msg.angle_min
        angle_inc = msg.angle_increment

        gap_points = []

        for i, r in enumerate(ranges):
            if r > self.gap_threshold:
                angle = angle_min + i * angle_inc
                # Only consider gaps in front of the robot (within 90 degrees)
                if abs(angle) < math.pi / 2:
                    x = r * math.cos(angle)
                    y = r * math.sin(angle)
                    gap_points.append((x, y, i))

        if len(gap_points) < 5:
            return

        # cluster consecutive gaps
        clusters = []
        current_cluster = [gap_points[0]]

        for i in range(1, len(gap_points)):
            if gap_points[i][2] - gap_points[i-1][2] < 5:
                current_cluster.append(gap_points[i])
            else:
                clusters.append(current_cluster)
                current_cluster = [gap_points[i]]

        clusters.append(current_cluster)

        # get the largest cluster
        best_cluster = max(clusters, key=lambda c: len(c))

        # calculate width
        p1 = best_cluster[0]
        p2 = best_cluster[-1]

        width = math.hypot(p2[0] - p1[0], p2[1] - p1[1])

        if width < self.min_gap_width:
            return

        # center
        cx = (p1[0] + p2[0]) / 2.0
        cy = (p1[1] + p2[1]) / 2.0

        msg_out = PointStamped()
        msg_out.header = msg.header
        msg_out.point.x = cx
        msg_out.point.y = cy
        msg_out.point.z = 0.0

        self.pub.publish(msg_out)

def main():
    rclpy.init()
    node = ElevatorDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()