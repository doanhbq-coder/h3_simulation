#!/usr/bin/env python3
"""
Subscribe to a raw camera image and republish as mono8.
Used when the camera hardware does not natively output mono8.

YUYV layout per macro-pixel (4 bytes = 2 pixels):
  byte 0: Y0   byte 1: U   byte 2: Y1   byte 3: V
Y values live at even byte indices → slice [::2].
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Image
import cv2
import numpy as np


class YuyvToMono8(Node):
    def __init__(self):
        super().__init__('yuyv_to_mono8')

        self.declare_parameter('input_topic',  '/camera/imx335/image_raw')
        self.declare_parameter('output_topic', '/camera/imx335/image_mono8')

        in_topic  = self.get_parameter('input_topic').get_parameter_value().string_value
        out_topic = self.get_parameter('output_topic').get_parameter_value().string_value

        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.pub = self.create_publisher(Image, out_topic, sensor_qos)
        self.sub = self.create_subscription(Image, in_topic, self.callback, sensor_qos)
        self.get_logger().info(f'Converting {in_topic} → {out_topic} (mono8)')

    def callback(self, msg: Image):
        enc = msg.encoding.lower()
        raw = np.frombuffer(msg.data, dtype=np.uint8)

        if enc in ('yuyv', 'yuv422_yuy2', 'yuv422p'):
            # YUYV: Y at every even byte — fastest path, no cvtColor needed
            gray = raw[::2].reshape(msg.height, msg.width)

        elif enc in ('uyvy',):
            # UYVY: Y at every odd byte
            gray = raw[1::2].reshape(msg.height, msg.width)

        elif enc in ('bgr8', 'bgra8'):
            ch = 4 if enc == 'bgra8' else 3
            arr = raw.reshape(msg.height, msg.width, ch)
            gray = cv2.cvtColor(arr, cv2.COLOR_BGR2GRAY)

        elif enc in ('rgb8', 'rgba8'):
            ch = 4 if enc == 'rgba8' else 3
            arr = raw.reshape(msg.height, msg.width, ch)
            gray = cv2.cvtColor(arr, cv2.COLOR_RGB2GRAY)

        elif enc == 'mono8':
            gray = raw.reshape(msg.height, msg.width)

        else:
            self.get_logger().warn(f'Unsupported encoding: {enc}', throttle_duration_sec=5.0)
            return

        out = Image()
        out.header       = msg.header
        out.height       = msg.height
        out.width        = msg.width
        out.encoding     = 'mono8'
        out.is_bigendian = 0
        out.step         = msg.width
        out.data         = gray.tobytes()
        self.pub.publish(out)


def main():
    rclpy.init()
    node = YuyvToMono8()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
