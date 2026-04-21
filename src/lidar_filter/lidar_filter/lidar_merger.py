#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math
from threading import Lock

class LidarMerger(Node):
    def __init__(self):
        super().__init__('lidar_merger')
        
        # Subscribers
        self.front_sub = self.create_subscription(
            LaserScan,
            'scan_front',
            self.front_callback,
            10)
        
        self.rear_sub = self.create_subscription(
            LaserScan,
            'scan_rear',
            self.rear_callback,
            10)
        
        # Publisher
        self.merged_pub = self.create_publisher(LaserScan, 'scan', 10)
        
        # Store latest scans with timestamp for synchronization
        self.front_scan = None
        self.rear_scan = None
        self.lock = Lock()
        
        # Timeout for considering scans as synchronized (100ms)
        self.sync_timeout_ns = 100_000_000
        
        self.get_logger().info('Lidar Merger Node Started')
        self.get_logger().info('Front lidar angle range: -1.6 to 3.1 rad')
        self.get_logger().info('Rear lidar angle range: -4.7 to 0.0 rad')
    
    def front_callback(self, msg):
        with self.lock:
            self.front_scan = msg
            self.try_merge_and_publish()
    
    def rear_callback(self, msg):
        with self.lock:
            self.rear_scan = msg
            self.try_merge_and_publish()
    
    def try_merge_and_publish(self):
        """Try to merge scans if both are available and synchronized"""
        if self.front_scan is None or self.rear_scan is None:
            return
        
        # Check if scans are approximately synchronized (within timeout)
        time_diff = abs(self.front_scan.header.stamp.sec - self.rear_scan.header.stamp.sec) * 1_000_000_000
        time_diff += abs(self.front_scan.header.stamp.nanosec - self.rear_scan.header.stamp.nanosec)
        
        if time_diff > self.sync_timeout_ns:
            self.get_logger().debug(f'Scans not synchronized: {time_diff} ns apart')
            return
        
        # Create merged scan
        merged = LaserScan()
        # Use the newer timestamp
        if self.front_scan.header.stamp.sec > self.rear_scan.header.stamp.sec or \
           (self.front_scan.header.stamp.sec == self.rear_scan.header.stamp.sec and \
            self.front_scan.header.stamp.nanosec > self.rear_scan.header.stamp.nanosec):
            merged.header.stamp = self.front_scan.header.stamp
        else:
            merged.header.stamp = self.rear_scan.header.stamp
        
        merged.header.frame_id = 'laser_frame'  # Base frame for merged scan
        
        # Full 360 degree scan with 0.5 degree resolution for better coverage
        merged.angle_min = -math.pi
        merged.angle_max = math.pi
        merged.angle_increment = math.pi / 360.0  # 0.5 degree resolution
        merged.time_increment = 0.0
        merged.scan_time = 0.05  # 20 Hz = 0.05s
        merged.range_min = 0.05
        merged.range_max = 25.0
        
        num_readings = int((merged.angle_max - merged.angle_min) / merged.angle_increment) + 1
        merged.ranges = [float('inf')] * num_readings
        merged.intensities = [0.0] * num_readings
        
        # Merge front scan
        self.merge_scan(merged, self.front_scan, 'front')
        
        # Merge rear scan
        self.merge_scan(merged, self.rear_scan, 'rear')
        
        self.merged_pub.publish(merged)
    
    def merge_scan(self, merged, scan, source):
        """Merge a single scan into the merged scan"""
        angle = scan.angle_min
        
        for i, r in enumerate(scan.ranges):
            # Skip invalid readings
            if r < scan.range_min or r > scan.range_max or math.isinf(r) or math.isnan(r):
                angle += scan.angle_increment
                continue
            
            # Convert to global angle based on source
            if source == 'front':
                # Front lidar: angle range -1.6 to 3.1 rad, already pointing forward
                global_angle = angle
            elif source == 'rear':
                # Rear lidar: angle range -4.7 to 0.0 rad (pointing backwards)
                # Convert rear angles to front coordinates:
                # Rear sensor frame: -4.7 to 0.0 rad
                # This represents a scan looking backward with left at -4.7 (-269 deg) and right at 0
                # In the base_link frame (front-facing), the rear 180 is at pi, so:
                global_angle = angle + math.pi
            else:
                continue
            
            # Clamp to [-pi, pi]
            while global_angle > math.pi:
                global_angle -= 2 * math.pi
            while global_angle < -math.pi:
                global_angle += 2 * math.pi
            
            # Find index in merged scan
            index = int((global_angle - merged.angle_min) / merged.angle_increment + 0.5)
            if 0 <= index < len(merged.ranges):
                if r < merged.ranges[index]:  # Take closer reading
                    merged.ranges[index] = r
                    if i < len(scan.intensities):
                        merged.intensities[index] = scan.intensities[i]
            
            angle += scan.angle_increment

def main(args=None):
    rclpy.init(args=args)
    node = LidarMerger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
