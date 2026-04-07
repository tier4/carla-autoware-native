#!/usr/bin/env python3
"""Measure LiDAR topic publish rate and latency.

Usage:
    # Measure single topic:
    ros2 run --prefix 'python3' -- measure_lidar_rate.py /sensing/lidar/top/pointcloud_raw_ex

    # Or directly:
    python3 measure_lidar_rate.py /sensing/lidar/top/pointcloud_raw_ex

    # Measure two topics (CARLA vs RGL comparison):
    python3 measure_lidar_rate.py /sensing/lidar/top/pointcloud_raw_ex /sensing/lidar/top/rgl_pointcloud
"""

import sys
import time
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2


class TopicStats:
    def __init__(self, topic_name):
        self.topic_name = topic_name
        self.count = 0
        self.first_time = None
        self.last_time = None
        self.intervals = []
        self.widths = []
        self.lock = threading.Lock()

    def on_message(self, msg):
        now = time.monotonic()
        with self.lock:
            if self.first_time is None:
                self.first_time = now
            if self.last_time is not None:
                self.intervals.append(now - self.last_time)
            self.last_time = now
            self.count += 1
            self.widths.append(msg.width)

    def report(self):
        with self.lock:
            if self.count == 0:
                return f"  {self.topic_name}: no messages received"

            elapsed = self.last_time - self.first_time if self.count > 1 else 0
            avg_hz = (self.count - 1) / elapsed if elapsed > 0 else 0

            avg_interval = sum(self.intervals) / len(self.intervals) if self.intervals else 0
            min_interval = min(self.intervals) if self.intervals else 0
            max_interval = max(self.intervals) if self.intervals else 0

            avg_width = sum(self.widths) / len(self.widths) if self.widths else 0

            lines = [
                f"  {self.topic_name}:",
                f"    Messages: {self.count}",
                f"    Rate: {avg_hz:.1f} Hz",
                f"    Interval: avg={avg_interval*1000:.1f}ms min={min_interval*1000:.1f}ms max={max_interval*1000:.1f}ms",
                f"    Width (points): avg={avg_width:.0f}",
            ]
            return "\n".join(lines)


class LidarRateMeasurer(Node):
    def __init__(self, topics):
        super().__init__('lidar_rate_measurer')
        self.stats = {}

        # Use best_effort to match both CARLA and RGL publishers
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        for topic in topics:
            stats = TopicStats(topic)
            self.stats[topic] = stats
            self.create_subscription(
                PointCloud2,
                topic,
                stats.on_message,
                qos
            )
            self.get_logger().info(f"Subscribing to {topic}")

    def print_report(self):
        print("\n" + "=" * 60)
        print("LiDAR Topic Rate Measurement")
        print("=" * 60)
        for stats in self.stats.values():
            print(stats.report())
        print("=" * 60)


def main():
    if len(sys.argv) < 2:
        print("Usage: python3 measure_lidar_rate.py <topic1> [topic2]")
        print("Example: python3 measure_lidar_rate.py /sensing/lidar/top/pointcloud_raw_ex")
        sys.exit(1)

    topics = sys.argv[1:]
    duration = 10  # seconds

    rclpy.init()
    node = LidarRateMeasurer(topics)

    print(f"Measuring for {duration} seconds...")

    start = time.monotonic()
    try:
        while time.monotonic() - start < duration:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass

    node.print_report()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
