#!/usr/bin/env python3
"""
pitag_setpose – Set robot initial pose via ceiling PiTag detection.

Math (all transforms are 4×4 homogeneous matrices):
    T_map_base = T_map_tag  ×  T_tag_camera  ×  T_camera_base
               = T_map_tag  ×  inv(T_camera_tag)  ×  inv(T_base_camera)

Where:
  T_map_tag      – known from config (user-measured position of each tag on ceiling)
  T_camera_tag   – measured by cob_fiducials (detection.pose.pose)
  T_base_camera  – from TF tree (base_link → camera frame)

Outputs (both run on every valid detection):
  1. POST {api_host}/cmd/reloc_pose  →  {"x":…,"y":…,"theta":…}   (real robot API)
  2. /initialpose topic              →  PoseWithCovarianceStamped   (simulation AMCL)
"""

import json
import math
import os
import threading
import urllib.error
import urllib.request

import numpy as np
import rclpy
import tf2_ros
import yaml
from geometry_msgs.msg import PoseWithCovarianceStamped
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from scipy.spatial.transform import Rotation

from cob_object_detection_msgs.msg import DetectionArray


def _rpy_deg_to_matrix(roll_deg: float, pitch_deg: float, yaw_deg: float) -> np.ndarray:
    return Rotation.from_euler(
        'xyz',
        [math.radians(roll_deg), math.radians(pitch_deg), math.radians(yaw_deg)]
    ).as_matrix()


def _make_transform(x: float, y: float, z: float,
                    roll_deg: float, pitch_deg: float, yaw_deg: float) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = _rpy_deg_to_matrix(roll_deg, pitch_deg, yaw_deg)
    T[0, 3] = x
    T[1, 3] = y
    T[2, 3] = z
    return T


def _pose_to_matrix(pos, ori) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = Rotation.from_quat([ori.x, ori.y, ori.z, ori.w]).as_matrix()
    T[0, 3] = pos.x
    T[1, 3] = pos.y
    T[2, 3] = pos.z
    return T


def _tf_to_matrix(tf_transform) -> np.ndarray:
    t = tf_transform.translation
    r = tf_transform.rotation
    return _pose_to_matrix(t, r)


class SetPoseNode(Node):
    def __init__(self):
        super().__init__('pitag_setpose')

        self.declare_parameter(
            'config_file', '',
            ParameterDescriptor(description='Absolute path to pitags.yaml config file'))
        self.declare_parameter(
            'detection_topic', '/fiducials/detect_fiducials',
            ParameterDescriptor(description='DetectionArray topic from cob_fiducials'))
        self.declare_parameter(
            'initialpose_topic', '/initialpose',
            ParameterDescriptor(description='Topic to publish PoseWithCovarianceStamped'))
        self.declare_parameter(
            'api_host', '',
            ParameterDescriptor(
                description='Base URL of robot API, e.g. http://192.168.1.100 '
                            '(empty = disabled). Calls POST {api_host}/cmd/reloc_pose'))
        self.declare_parameter(
            'api_timeout', 3.0,
            ParameterDescriptor(description='HTTP request timeout in seconds'))

        config_file = self.get_parameter('config_file').value
        if not config_file or not os.path.isfile(config_file):
            self.get_logger().fatal(
                f'config_file parameter not set or file not found: "{config_file}"')
            raise RuntimeError('config_file missing')

        with open(config_file) as f:
            cfg = yaml.safe_load(f)

        self._map_frame: str = cfg.get('map_frame', 'map')
        self._base_frame: str = cfg.get('base_frame', 'base_link')
        self._min_score: float = float(cfg.get('min_score', 0.0))
        self._debounce_sec: float = float(cfg.get('debounce_seconds', 2.0))
        self._covariance: list = cfg.get('covariance', _default_covariance())

        self._api_host: str = self.get_parameter('api_host').value.rstrip('/')
        self._api_timeout: float = self.get_parameter('api_timeout').value

        # Build T_map_tag dict keyed by integer tag id
        self._tag_transforms: dict[int, np.ndarray] = {}
        for tag in cfg.get('tags', []):
            tid = int(tag['id'])
            T = _make_transform(
                float(tag.get('x', 0.0)),
                float(tag.get('y', 0.0)),
                float(tag.get('z', 0.0)),
                float(tag.get('roll', 0.0)),
                float(tag.get('pitch', 0.0)),
                float(tag.get('yaw', 0.0)),
            )
            self._tag_transforms[tid] = T
            self.get_logger().info(
                f'Loaded tag {tid}: map pos=({tag.get("x", 0):.3f}, '
                f'{tag.get("y", 0):.3f}, {tag.get("z", 0):.3f})')

        if not self._tag_transforms:
            self.get_logger().warn('No tags loaded from config – node will not set poses')

        self._last_publish_time: float = 0.0

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self._pose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            self.get_parameter('initialpose_topic').value,
            10)

        det_topic = self.get_parameter('detection_topic').value
        self._sub = self.create_subscription(
            DetectionArray, det_topic, self._detection_cb, 10)

        api_status = self._api_host if self._api_host else 'disabled'
        self.get_logger().info(
            f'pitag_setpose ready | '
            f'map={self._map_frame} base={self._base_frame} '
            f'tags={list(self._tag_transforms.keys())} '
            f'api={api_status}')

    # ──────────────────────────────────────────────────────────────────────────

    def _detection_cb(self, msg: DetectionArray) -> None:
        now_sec = self.get_clock().now().nanoseconds * 1e-9
        if now_sec - self._last_publish_time < self._debounce_sec:
            return

        for det in msg.detections:
            if det.id not in self._tag_transforms:
                continue

            if self._min_score > 0.0 and det.score > self._min_score:
                continue

            camera_frame = det.pose.header.frame_id
            if not camera_frame:
                self.get_logger().warn('Detection has empty frame_id, skipping')
                continue

            # T_camera_tag from detection
            T_camera_tag = _pose_to_matrix(
                det.pose.pose.position, det.pose.pose.orientation)

            # T_base_camera from TF tree
            try:
                tf_stamped = self._tf_buffer.lookup_transform(
                    self._base_frame, camera_frame, rclpy.time.Time())
                T_base_camera = _tf_to_matrix(tf_stamped.transform)
            except (tf2_ros.LookupException,
                    tf2_ros.ConnectivityException,
                    tf2_ros.ExtrapolationException) as e:
                self.get_logger().warn(
                    f'TF {self._base_frame} → {camera_frame} not available: {e}')
                continue

            # T_map_base = T_map_tag × inv(T_camera_tag) × inv(T_base_camera)
            T_map_tag = self._tag_transforms[det.id]
            T_map_base = T_map_tag @ np.linalg.inv(T_camera_tag) @ np.linalg.inv(T_base_camera)

            # Extract 2D pose
            x = T_map_base[0, 3]
            y = T_map_base[1, 3]
            yaw = Rotation.from_matrix(T_map_base[:3, :3]).as_euler('zyx')[0]

            self._last_publish_time = now_sec

            # ── 1. Gọi API robot thật ─────────────────────────────────────────
            if self._api_host:
                threading.Thread(
                    target=self._call_reloc_api,
                    args=(x, y, yaw),
                    daemon=True,
                ).start()

            # ── 2. Publish /initialpose cho simulation AMCL ───────────────────
            q = Rotation.from_euler('z', yaw).as_quat()  # [qx, qy, qz, qw]
            pose_msg = PoseWithCovarianceStamped()
            pose_msg.header.stamp = msg.header.stamp
            pose_msg.header.frame_id = self._map_frame
            pose_msg.pose.pose.position.x = x
            pose_msg.pose.pose.position.y = y
            pose_msg.pose.pose.position.z = 0.0
            pose_msg.pose.pose.orientation.x = q[0]
            pose_msg.pose.pose.orientation.y = q[1]
            pose_msg.pose.pose.orientation.z = q[2]
            pose_msg.pose.pose.orientation.w = q[3]
            pose_msg.pose.covariance = self._covariance
            self._pose_pub.publish(pose_msg)

            self.get_logger().info(
                f'[tag {det.id}] x={x:.3f} y={y:.3f} '
                f'yaw={math.degrees(yaw):.1f}° score={det.score:.1f}px')
            break  # first valid detection per callback

    def _call_reloc_api(self, x: float, y: float, theta: float) -> None:
        url = f'{self._api_host}/cmd/reloc_pose'
        body = json.dumps({'x': round(x, 4), 'y': round(y, 4), 'theta': round(theta, 6)})
        req = urllib.request.Request(
            url,
            data=body.encode(),
            headers={'Content-Type': 'application/json'},
            method='POST',
        )
        try:
            with urllib.request.urlopen(req, timeout=self._api_timeout) as resp:
                result = json.loads(resp.read().decode())
                status = result.get('status', '?')
                self.get_logger().info(f'reloc_pose API → {status}  ({url})')
        except urllib.error.HTTPError as e:
            self.get_logger().error(f'reloc_pose API HTTP {e.code}: {e.reason}  ({url})')
        except urllib.error.URLError as e:
            self.get_logger().error(f'reloc_pose API unreachable: {e.reason}  ({url})')
        except Exception as e:
            self.get_logger().error(f'reloc_pose API error: {e}  ({url})')


def _default_covariance() -> list:
    cov = [0.0] * 36
    cov[0] = 0.25
    cov[7] = 0.25
    cov[35] = 0.0685
    return cov


def main(args=None):
    rclpy.init(args=args)
    node = SetPoseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
