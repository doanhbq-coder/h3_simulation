import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class Mover(Node):
    def __init__(self):
        super().__init__('mover')

        self.pub1 = self.create_publisher(Twist, '/sphere1/cmd_vel', 10)
        self.pub2 = self.create_publisher(Twist, '/sphere2/cmd_vel', 10)
        self.pub3 = self.create_publisher(Twist, '/sphere3/cmd_vel', 10)
        self.pub4 = self.create_publisher(Twist, '/sphere4/cmd_vel', 10)
        self.pub5 = self.create_publisher(Twist, '/sphere5/cmd_vel', 10)
        self.pub6 = self.create_publisher(Twist, '/sphere6/cmd_vel', 10)
        self.pub7 = self.create_publisher(Twist, '/sphere7/cmd_vel', 10)
        self.pub8 = self.create_publisher(Twist, '/sphere8/cmd_vel', 10)
        self.pub9 = self.create_publisher(Twist, '/sphere9/cmd_vel', 10)
        self.pub10 = self.create_publisher(Twist, '/sphere10/cmd_vel', 10)

        self.timer = self.create_timer(0.1, self.loop)
        self.t = 0.0

    def loop(self):
        msg1 = Twist()
        msg2 = Twist()
        msg3 = Twist()
        msg4 = Twist()
        msg5 = Twist()
        msg6 = Twist()
        msg7 = Twist()
        msg8 = Twist()
        msg9 = Twist()
        msg10 = Twist()
        # Đổi hướng mỗi 5 giây
        if int(self.t) % 14 < 7:
            msg1.linear.x = -0.4
            msg2.linear.x = 0.4
            msg3.linear.x = -0.4
            msg4.linear.x = 0.4
            msg5.linear.x = -0.4
            msg6.linear.x = 0.4
            msg7.linear.x = -0.4
            msg8.linear.x = 0.4
            msg9.linear.x = -0.4
            msg10.linear.x = 0.4
        else:
            msg1.linear.x = 0.4
            msg2.linear.x = -0.4
            msg3.linear.x = 0.4
            msg4.linear.x = -0.4
            msg5.linear.x = 0.4
            msg6.linear.x = -0.4
            msg7.linear.x = 0.4
            msg8.linear.x = -0.4
            msg9.linear.x = 0.4
            msg10.linear.x = -0.4

        self.pub1.publish(msg1)
        self.pub2.publish(msg2)
        self.pub3.publish(msg3)
        self.pub4.publish(msg4)
        self.pub5.publish(msg5)
        self.pub6.publish(msg6)
        self.pub7.publish(msg7)
        self.pub8.publish(msg8)
        self.pub9.publish(msg9)
        self.pub10.publish(msg10)

        self.t += 0.1


def main():
    rclpy.init()
    node = Mover()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()