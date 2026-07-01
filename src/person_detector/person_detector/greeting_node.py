#!/usr/bin/env python3
"""
Greeting node for receptionist robot.

Subscribes to /greeting_trigger (JSON String) and performs greeting actions.
Extend the _do_greeting() method to integrate with TTS, LED, or sound hardware.
"""

import json
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String


class GreetingNode(Node):

    def __init__(self):
        super().__init__('greeting_node')

        self.declare_parameter('greeting_text', 'Xin chào! Tôi có thể giúp gì cho bạn?')
        self.declare_parameter('tts_topic', '/tts/text')

        self.greeting_text = self.get_parameter('greeting_text').value
        self.tts_topic = self.get_parameter('tts_topic').value

        self.trigger_sub = self.create_subscription(
            String, '/greeting_trigger', self.on_greeting_trigger, 10)

        self.detected_sub = self.create_subscription(
            Bool, '/person_detected', self.on_person_detected, 10)

        # Publish to TTS if available
        self.tts_pub = self.create_publisher(String, self.tts_topic, 10)

        self.get_logger().info('Greeting node ready')

    def on_greeting_trigger(self, msg: String):
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            payload = {}

        dist = payload.get('distance', '?')
        pos = payload.get('position', {})
        self.get_logger().info(
            f'[GREETING] Person detected at {dist}m '
            f'(x={pos.get("x","?")}, y={pos.get("y","?")})'
        )

        self._do_greeting(dist)

    def on_person_detected(self, msg: Bool):
        if not msg.data:
            self.get_logger().debug('Person left the detection zone')

    def _do_greeting(self, distance):
        """
        Perform the greeting action. Extend this method to:
        - Play a sound file: subprocess.run(['aplay', 'hello.wav'])
        - Call TTS API
        - Trigger LED animation
        - Navigate toward person
        """
        tts_msg = String()
        tts_msg.data = self.greeting_text
        self.tts_pub.publish(tts_msg)

        self.get_logger().info(f'>> {self.greeting_text}')


def main(args=None):
    rclpy.init(args=args)
    node = GreetingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
