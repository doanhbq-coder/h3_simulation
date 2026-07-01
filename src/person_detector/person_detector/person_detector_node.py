#!/usr/bin/env python3
"""
Person detector for receptionist robot.

Algorithm: Leg-pair detection on 2D LiDAR data.
- LiDAR mounted at ~25cm height will see human legs as two isolated cylindrical clusters.
- Each cluster (leg) is 8-25cm wide; the two legs are 10-60cm apart (stance width).
- Walls and furniture produce either very large clusters or are not in pairs → filtered out.
- Temporal filter (N consecutive detections) reduces false positives from transient noise.
- Movement check (optional): static clusters are likely furniture, not people.
"""

import math
import json
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String
from visualization_msgs.msg import Marker, MarkerArray


class PersonDetectorNode(Node):

    def __init__(self):
        super().__init__('person_detector')

        # --- Parameters ---
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('detection_range', 2.0)        # meters
        self.declare_parameter('leg_min_width', 0.05)         # 5cm  - narrowest leg cross-section
        self.declare_parameter('leg_max_width', 0.25)         # 25cm - widest leg cross-section
        self.declare_parameter('min_leg_points', 2)           # min scan rays per leg cluster
        self.declare_parameter('max_leg_points', 40)          # max rays - avoids large furniture
        self.declare_parameter('max_cluster_gap', 0.12)       # 12cm - gap to split clusters
        self.declare_parameter('min_stance_width', 0.10)      # 10cm between two legs
        self.declare_parameter('max_stance_width', 0.65)      # 65cm max human stance
        self.declare_parameter('greeting_cooldown', 8.0)      # seconds between greetings
        self.declare_parameter('consecutive_detections', 4)   # frames required to confirm
        self.declare_parameter('require_movement', False)     # True = ignore static detections
        self.declare_parameter('movement_threshold', 0.05)    # 5cm movement to count

        self._load_params()

        # --- State ---
        self.last_greeting_time = 0.0
        self.consecutive_count = 0
        self.person_present = False
        self.tracked_persons = {}
        self.track_id_counter = 0

        # --- ROS I/O ---
        self.scan_sub = None
        self.detected_pub = self.create_publisher(Bool, '/person_detected', 10)
        self.greeting_pub = self.create_publisher(String, '/greeting_trigger', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/detected_persons_markers', 10)

        self._create_subscription(self.scan_topic)

        # Re-subscribe automatically if scan_topic param is changed at runtime
        self.add_on_set_parameters_callback(self._on_param_change)

        self.get_logger().info(
            f'Person Detector started | scan: {self.scan_topic} | '
            f'range: {self.detection_range}m | cooldown: {self.greeting_cooldown}s'
        )

    def _load_params(self):
        self.scan_topic = self.get_parameter('scan_topic').value
        self.detection_range = self.get_parameter('detection_range').value
        self.leg_min_width = self.get_parameter('leg_min_width').value
        self.leg_max_width = self.get_parameter('leg_max_width').value
        self.min_leg_points = self.get_parameter('min_leg_points').value
        self.max_leg_points = self.get_parameter('max_leg_points').value
        self.max_cluster_gap = self.get_parameter('max_cluster_gap').value
        self.min_stance_width = self.get_parameter('min_stance_width').value
        self.max_stance_width = self.get_parameter('max_stance_width').value
        self.greeting_cooldown = self.get_parameter('greeting_cooldown').value
        self.required_consecutive = self.get_parameter('consecutive_detections').value
        self.require_movement = self.get_parameter('require_movement').value
        self.movement_threshold = self.get_parameter('movement_threshold').value

    def _create_subscription(self, topic: str):
        if self.scan_sub is not None:
            self.destroy_subscription(self.scan_sub)
        self.scan_sub = self.create_subscription(
            LaserScan, topic, self.scan_callback, 10)
        self.get_logger().info(f'Subscribed to {topic}')

    def _on_param_change(self, params):
        from rcl_interfaces.msg import SetParametersResult
        for p in params:
            if p.name == 'scan_topic' and p.value != self.scan_topic:
                self.scan_topic = p.value
                self._create_subscription(self.scan_topic)
            elif p.name == 'detection_range':
                self.detection_range = p.value
            elif p.name == 'greeting_cooldown':
                self.greeting_cooldown = p.value
            elif p.name == 'consecutive_detections':
                self.required_consecutive = p.value
            elif p.name == 'leg_min_width':
                self.leg_min_width = p.value
            elif p.name == 'leg_max_width':
                self.leg_max_width = p.value
            elif p.name == 'min_stance_width':
                self.min_stance_width = p.value
            elif p.name == 'max_stance_width':
                self.max_stance_width = p.value
            elif p.name == 'max_cluster_gap':
                self.max_cluster_gap = p.value
        return SetParametersResult(successful=True)

    # ------------------------------------------------------------------
    # Main pipeline
    # ------------------------------------------------------------------

    def scan_callback(self, msg: LaserScan):
        self._frame_count = getattr(self, '_frame_count', 0) + 1

        points = self._scan_to_cartesian(msg)
        clusters = self._cluster_points(points)
        clusters = self._merge_wraparound(clusters)
        leg_clusters = self._filter_leg_clusters(clusters)
        persons = self._find_person_pairs(leg_clusters)

        if self.require_movement:
            persons = self._filter_static(persons)

        person_detected = len(persons) > 0

        # Log every 50 frames so we can see what's happening
        if self._frame_count % 50 == 1:
            self.get_logger().info(
                f'[frame {self._frame_count}] pts={len(points)} '
                f'clusters={len(clusters)} legs={len(leg_clusters)} '
                f'persons={len(persons)} consecutive={self.consecutive_count}'
            )

        if person_detected:
            self.consecutive_count += 1
        else:
            self.consecutive_count = 0
            if self.person_present:
                self.person_present = False
                self._publish_detected(False)
                self.get_logger().info('Person left detection zone')

        confirmed = self.consecutive_count >= self.required_consecutive

        if confirmed and not self.person_present:
            self.person_present = True
            self._publish_detected(True)
            self._maybe_trigger_greeting(persons)

        self._publish_markers(persons, msg.header)

    # ------------------------------------------------------------------
    # Step 1: polar → Cartesian, filter by range
    # ------------------------------------------------------------------

    def _scan_to_cartesian(self, msg: LaserScan):
        """Return list of (x, y) within detection_range."""
        pts = []
        angle = msg.angle_min
        for r in msg.ranges:
            valid = (
                msg.range_min < r < min(self.detection_range, msg.range_max)
                and not math.isinf(r) and not math.isnan(r)
            )
            if valid:
                pts.append((r * math.cos(angle), r * math.sin(angle)))
            angle += msg.angle_increment
        return pts

    # ------------------------------------------------------------------
    # Step 2: sequential angle-ordered clustering
    # ------------------------------------------------------------------

    def _cluster_points(self, points):
        """Group consecutive points separated by < max_cluster_gap."""
        if not points:
            return []

        clusters = []
        current = [points[0]]

        for i in range(1, len(points)):
            dx = points[i][0] - points[i-1][0]
            dy = points[i][1] - points[i-1][1]
            if math.hypot(dx, dy) < self.max_cluster_gap:
                current.append(points[i])
            else:
                clusters.append(current)
                current = [points[i]]
        clusters.append(current)
        return clusters

    def _merge_wraparound(self, clusters):
        """Merge first and last cluster if they are spatially adjacent (360° scan wrap)."""
        if len(clusters) < 2:
            return clusters
        first, last = clusters[0], clusters[-1]
        dx = first[0][0] - last[-1][0]
        dy = first[0][1] - last[-1][1]
        if math.hypot(dx, dy) < self.max_cluster_gap:
            merged = last + first
            return [merged] + clusters[1:-1]
        return clusters

    # ------------------------------------------------------------------
    # Step 3: classify clusters as leg-like
    # ------------------------------------------------------------------

    def _cluster_width(self, cluster):
        """Max pairwise distance within cluster (bounding diameter)."""
        if len(cluster) < 2:
            return 0.0
        max_d = 0.0
        for i in range(len(cluster)):
            for j in range(i + 1, len(cluster)):
                d = math.hypot(cluster[i][0] - cluster[j][0], cluster[i][1] - cluster[j][1])
                if d > max_d:
                    max_d = d
        return max_d

    def _cluster_center(self, cluster):
        n = len(cluster)
        return sum(p[0] for p in cluster) / n, sum(p[1] for p in cluster) / n

    def _filter_leg_clusters(self, clusters):
        """Keep only clusters that could be a human leg cross-section."""
        legs = []
        for c in clusters:
            n = len(c)
            if n < self.min_leg_points or n > self.max_leg_points:
                continue
            w = self._cluster_width(c)
            if self.leg_min_width <= w <= self.leg_max_width:
                legs.append(c)
        return legs

    # ------------------------------------------------------------------
    # Step 4: pair legs → persons
    # ------------------------------------------------------------------

    def _find_person_pairs(self, leg_clusters):
        """Find pairs of leg-like clusters at human stance distance."""
        persons = []
        used = set()

        for i in range(len(leg_clusters)):
            if i in used:
                continue
            ci = self._cluster_center(leg_clusters[i])

            best_j = None
            best_stance = float('inf')

            for j in range(i + 1, len(leg_clusters)):
                if j in used:
                    continue
                cj = self._cluster_center(leg_clusters[j])
                stance = math.hypot(ci[0] - cj[0], ci[1] - cj[1])

                if self.min_stance_width <= stance <= self.max_stance_width:
                    if stance < best_stance:
                        best_stance = stance
                        best_j = j

            if best_j is not None:
                cj = self._cluster_center(leg_clusters[best_j])
                mx = (ci[0] + cj[0]) / 2.0
                my = (ci[1] + cj[1]) / 2.0
                dist = math.hypot(mx, my)

                if dist <= self.detection_range:
                    persons.append({
                        'x': mx, 'y': my,
                        'distance': round(dist, 3),
                        'stance': round(best_stance, 3),
                        'leg1': ci, 'leg2': cj,
                    })
                    used.add(i)
                    used.add(best_j)

        return persons

    # ------------------------------------------------------------------
    # Step 5 (optional): filter out static detections (furniture)
    # ------------------------------------------------------------------

    def _filter_static(self, persons):
        """Remove person candidates that haven't moved between frames."""
        new_tracked = {}
        moving = []

        for p in persons:
            px, py = p['x'], p['y']
            matched_id = None
            min_dist = self.max_stance_width

            for tid, (tx, ty, frames, moved) in self.tracked_persons.items():
                d = math.hypot(px - tx, py - ty)
                if d < min_dist:
                    min_dist = d
                    matched_id = tid

            if matched_id is not None:
                tx, ty, frames, moved = self.tracked_persons[matched_id]
                total_movement = math.hypot(px - tx, py - ty)
                is_moving = moved or (total_movement > self.movement_threshold)
                new_tracked[matched_id] = (px, py, frames + 1, is_moving)
                if is_moving:
                    moving.append(p)
            else:
                new_tracked[self.track_id_counter] = (px, py, 1, False)
                self.track_id_counter += 1

        self.tracked_persons = new_tracked
        return moving

    # ------------------------------------------------------------------
    # Publishing
    # ------------------------------------------------------------------

    def _publish_detected(self, state: bool):
        msg = Bool()
        msg.data = state
        self.detected_pub.publish(msg)

    def _maybe_trigger_greeting(self, persons):
        now = time.time()
        if now - self.last_greeting_time < self.greeting_cooldown:
            return

        self.last_greeting_time = now
        nearest = min(persons, key=lambda p: p['distance'])

        payload = {
            'event': 'person_detected',
            'distance': nearest['distance'],
            'position': {'x': round(nearest['x'], 2), 'y': round(nearest['y'], 2)},
            'timestamp': now,
        }
        msg = String()
        msg.data = json.dumps(payload)
        self.greeting_pub.publish(msg)

        self.get_logger().info(
            f'[GREETING] Person at {nearest["distance"]:.2f}m '
            f'(x={nearest["x"]:.2f}, y={nearest["y"]:.2f})'
        )

    def _publish_markers(self, persons, header):
        markers = MarkerArray()

        # Delete all previous markers first
        delete_all = Marker()
        delete_all.action = Marker.DELETEALL
        markers.markers.append(delete_all)

        for i, p in enumerate(persons):
            # Cylinder body
            body = Marker()
            body.header = header
            body.ns = 'person_body'
            body.id = i
            body.type = Marker.CYLINDER
            body.action = Marker.ADD
            body.pose.position.x = p['x']
            body.pose.position.y = p['y']
            body.pose.position.z = 0.9
            body.pose.orientation.w = 1.0
            body.scale.x = 0.4
            body.scale.y = 0.4
            body.scale.z = 1.8
            body.color.r = 0.1
            body.color.g = 0.9
            body.color.b = 0.3
            body.color.a = 0.55
            body.lifetime.sec = 1
            markers.markers.append(body)

            # Distance text label
            label = Marker()
            label.header = header
            label.ns = 'person_label'
            label.id = i
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x = p['x']
            label.pose.position.y = p['y']
            label.pose.position.z = 2.0
            label.scale.z = 0.18
            label.color.r = 1.0
            label.color.g = 1.0
            label.color.b = 1.0
            label.color.a = 1.0
            label.text = f'{p["distance"]:.2f}m'
            label.lifetime.sec = 1
            markers.markers.append(label)

        self.marker_pub.publish(markers)


def main(args=None):
    rclpy.init(args=args)
    node = PersonDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
