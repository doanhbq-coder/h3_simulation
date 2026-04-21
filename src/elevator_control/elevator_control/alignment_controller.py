import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PointStamped
import math

class ElevatorController(Node):

    def __init__(self):
        super().__init__('elevator_controller')

        self.sub = self.create_subscription(
            PointStamped,
            '/elevator_center',
            self.cb,
            10
        )

        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.k_ang = 0.5  # Reduced from 1.5 to prevent excessive spinning
        self.max_ang_vel = 0.3  # Maximum angular velocity limit
        self.forward_speed = 0.08
        self.stop_distance = 0.1  # Stop when within 10cm of center

    def cb(self, msg):
        x = msg.point.x
        y = msg.point.y

        # Calculate distance to center
        distance = math.sqrt(x**2 + y**2)

        # angle error
        angle = math.atan2(y, x)

        cmd = Twist()

        if distance < self.stop_distance:
            # Stop when close enough
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
        else:
            cmd.linear.x = self.forward_speed
            cmd.angular.z = max(-self.max_ang_vel, min(self.max_ang_vel, self.k_ang * angle))

        self.pub.publish(cmd)

def main():
    rclpy.init()
    node = ElevatorController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()