#!/usr/bin/env python3
"""
Scan debugger - chạy standalone để phân tích dữ liệu scan thực tế.

Dùng để:
  - Xem cluster nào đang được phát hiện
  - Check tại sao leg/person detection không trigger
  - Tìm ra ngưỡng tham số phù hợp

Usage:
  ros2 run person_detector scan_debugger
"""

import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class ScanDebugger(Node):

    def __init__(self):
        super().__init__('scan_debugger')

        self.declare_parameter('scan_topic', '/scan_top')
        self.declare_parameter('detection_range', 3.0)
        self.declare_parameter('max_cluster_gap', 0.12)
        self.declare_parameter('min_leg_points', 2)
        self.declare_parameter('max_leg_points', 80)
        self.declare_parameter('leg_min_width', 0.03)
        self.declare_parameter('leg_max_width', 0.40)
        self.declare_parameter('min_stance_width', 0.08)
        self.declare_parameter('max_stance_width', 0.80)

        self.scan_topic = self.get_parameter('scan_topic').value
        self.detection_range = self.get_parameter('detection_range').value
        self.max_cluster_gap = self.get_parameter('max_cluster_gap').value
        self.min_leg_points = self.get_parameter('min_leg_points').value
        self.max_leg_points = self.get_parameter('max_leg_points').value
        self.leg_min_width = self.get_parameter('leg_min_width').value
        self.leg_max_width = self.get_parameter('leg_max_width').value
        self.min_stance_width = self.get_parameter('min_stance_width').value
        self.max_stance_width = self.get_parameter('max_stance_width').value

        self.count = 0
        self.sub = self.create_subscription(
            LaserScan, self.scan_topic, self.callback, 10)

        self.get_logger().info(f'ScanDebugger listening on {self.scan_topic}')

    def callback(self, msg: LaserScan):
        self.count += 1
        # Print every 20 frames to avoid spam
        if self.count % 20 != 1:
            return

        pts = self._to_cartesian(msg)
        clusters = self._cluster(pts)

        print(f'\n{"="*60}')
        print(f'Frame #{self.count} | Points in {self.detection_range}m: {len(pts)}')
        print(f'Total clusters: {len(clusters)}')
        print()

        leg_candidates = []
        for i, c in enumerate(clusters):
            w = self._width(c)
            cx, cy = self._center(c)
            dist = math.hypot(cx, cy)
            n = len(c)

            is_leg = (
                self.min_leg_points <= n <= self.max_leg_points
                and self.leg_min_width <= w <= self.leg_max_width
            )

            marker = '✓ LEG' if is_leg else '✗'
            reason = ''
            if not is_leg:
                if n < self.min_leg_points:
                    reason = f'too few pts ({n}<{self.min_leg_points})'
                elif n > self.max_leg_points:
                    reason = f'too many pts ({n}>{self.max_leg_points})'
                elif w < self.leg_min_width:
                    reason = f'too narrow ({w*100:.1f}cm < {self.leg_min_width*100:.0f}cm)'
                elif w > self.leg_max_width:
                    reason = f'too wide ({w*100:.1f}cm > {self.leg_max_width*100:.0f}cm)'

            print(f'  Cluster {i:2d}: n={n:3d} pts | width={w*100:5.1f}cm | '
                  f'dist={dist:.2f}m | cx={cx:.2f} cy={cy:.2f} | {marker} {reason}')

            if is_leg:
                leg_candidates.append((i, c, cx, cy))

        print(f'\nLeg candidates: {len(leg_candidates)}')

        if len(leg_candidates) >= 2:
            print('\nPair analysis:')
            for a in range(len(leg_candidates)):
                for b in range(a+1, len(leg_candidates)):
                    ia, ca, cxa, cya = leg_candidates[a]
                    ib, cb, cxb, cyb = leg_candidates[b]
                    stance = math.hypot(cxa-cxb, cya-cyb)
                    mx = (cxa+cxb)/2
                    my = (cya+cyb)/2
                    mdist = math.hypot(mx, my)

                    valid = self.min_stance_width <= stance <= self.max_stance_width
                    marker = '✓ PERSON' if valid else '✗'
                    reason = ''
                    if not valid:
                        if stance < self.min_stance_width:
                            reason = f'stance too narrow ({stance*100:.1f}cm)'
                        else:
                            reason = f'stance too wide ({stance*100:.1f}cm)'

                    print(f'  Cluster {ia} + {ib}: stance={stance*100:.1f}cm | '
                          f'center dist={mdist:.2f}m | {marker} {reason}')
        else:
            print('  Not enough leg candidates to form a pair')


    def _to_cartesian(self, msg):
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

    def _cluster(self, pts):
        if not pts:
            return []
        clusters = []
        cur = [pts[0]]
        for i in range(1, len(pts)):
            if math.hypot(pts[i][0]-pts[i-1][0], pts[i][1]-pts[i-1][1]) < self.max_cluster_gap:
                cur.append(pts[i])
            else:
                clusters.append(cur)
                cur = [pts[i]]
        clusters.append(cur)
        return clusters

    def _width(self, c):
        mx = 0.0
        for i in range(len(c)):
            for j in range(i+1, len(c)):
                d = math.hypot(c[i][0]-c[j][0], c[i][1]-c[j][1])
                if d > mx:
                    mx = d
        return mx

    def _center(self, c):
        return sum(p[0] for p in c)/len(c), sum(p[1] for p in c)/len(c)


def main(args=None):
    rclpy.init(args=args)
    node = ScanDebugger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
