import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class Mover(Node):
    def __init__(self):
        super().__init__('mover')

        self.pub1 = self.create_publisher(Twist, '/sphere1/cmd_vel', 10)
        self.pub2 = self.create_publisher(Twist, '/sphere2/cmd_vel', 10)
        self.pub3 = self.create_publisher(Twist, '/sphere3/cmd_vel', 10)

        self.timer = self.create_timer(0.1, self.loop)
        self.t = 0.0

    def loop(self):
        msg1 = Twist()
        msg2 = Twist()
        msg3 = Twist()
        # Đổi hướng mỗi 5 giây
        if int(self.t) % 10 < 5:
            msg1.linear.x = -0.4
            msg2.linear.x = 0.4
            msg3.linear.x = -0.4
        else:
            msg1.linear.x = 0.4
            msg2.linear.x = -0.4
            msg3.linear.x = 0.4

        self.pub1.publish(msg1)
        self.pub2.publish(msg2)
        self.pub3.publish(msg3)

        self.t += 0.1


def main():
    rclpy.init()
    node = Mover()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()