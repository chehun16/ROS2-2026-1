#!/usr/bin/env python3
"""
slam_node.py

Node 1: SLAM Node
- Subscribes to /scan (LiDAR)
- slam_toolbox가 맵을 생성하도록 relay 및 상태 모니터링
- /map, /slam/status 를 publish
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


class SlamNode(Node):
    def __init__(self):
        super().__init__('slam_node')

        # Subscribers
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )

        # Publishers
        self.status_pub = self.create_publisher(String, '/slam/status', 10)

        # Timer: 1Hz 상태 publish
        self.timer = self.create_timer(1.0, self.publish_status)

        self.map_received = False
        self.scan_count = 0

        self.get_logger().info('SlamNode started')

    def scan_callback(self, msg: LaserScan):
        self.scan_count += 1

    def map_callback(self, msg: OccupancyGrid):
        self.map_received = True
        known = sum(1 for c in msg.data if c >= 0)
        total = len(msg.data)
        self.explored_ratio = known / total if total > 0 else 0.0

    def publish_status(self):
        status = String()
        status.data = (
            f'map_received={self.map_received}, '
            f'scan_count={self.scan_count}'
        )
        self.status_pub.publish(status)


def main(args=None):
    rclpy.init(args=args)
    node = SlamNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
