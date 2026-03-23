#!/usr/bin/python3
"""
viz_node.py

Node 5: Visualization Node
- Subscribes to /explorer/status, /perception/detections, /planner/state
- RViz MarkerArray로 탐지된 물체, 탐색 상태 텍스트 오버레이
- /viz/detected_objects   (MarkerArray) publish
- /viz/status_text        (MarkerArray) publish
"""

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import String, ColorRGBA
from geometry_msgs.msg import Vector3

from msgs.msg import DetectedObject, ExplorationStatus


class VizNode(Node):
    def __init__(self):
        super().__init__('viz_node')

        # Subscribers
        self.detection_sub = self.create_subscription(
            DetectedObject, '/perception/detections', self.detection_callback, 10
        )
        self.status_sub = self.create_subscription(
            ExplorationStatus, '/explorer/status', self.status_callback, 10
        )
        self.planner_state_sub = self.create_subscription(
            String, '/planner/state', self.planner_state_callback, 10
        )

        # Publishers
        self.obj_marker_pub = self.create_publisher(
            MarkerArray, '/viz/detected_objects', 10
        )
        self.text_marker_pub = self.create_publisher(
            MarkerArray, '/viz/status_text', 10
        )

        # State
        self.detected_objects: list[DetectedObject] = []
        self.exploration_status: ExplorationStatus = None
        self.planner_state = 'WAITING'

        # Timer: 2Hz 시각화 갱신
        self.timer = self.create_timer(0.5, self.publish_markers)

        self.get_logger().info('VizNode started')

    def detection_callback(self, msg: DetectedObject):
        self.detected_objects.append(msg)
        # 최대 20개 유지
        if len(self.detected_objects) > 20:
            self.detected_objects.pop(0)

    def status_callback(self, msg: ExplorationStatus):
        self.exploration_status = msg

    def planner_state_callback(self, msg: String):
        self.planner_state = msg.data

    def publish_markers(self):
        self._publish_object_markers()
        self._publish_status_text()

    def _publish_object_markers(self):
        marker_array = MarkerArray()

        for i, det in enumerate(self.detected_objects):
            # 구체 마커 (탐지 위치)
            sphere = Marker()
            sphere.header.frame_id = 'map'
            sphere.header.stamp = self.get_clock().now().to_msg()
            sphere.ns = 'detected_objects'
            sphere.id = i * 2
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD
            sphere.pose.position = det.position
            sphere.pose.orientation.w = 1.0
            sphere.scale = Vector3(x=0.4, y=0.4, z=0.4)
            sphere.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.9)
            sphere.lifetime.sec = 5
            marker_array.markers.append(sphere)

            # 텍스트 마커 (클래스명)
            text = Marker()
            text.header.frame_id = 'map'
            text.header.stamp = self.get_clock().now().to_msg()
            text.ns = 'detected_objects'
            text.id = i * 2 + 1
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position = det.position
            text.pose.position.z += 0.5
            text.pose.orientation.w = 1.0
            text.scale = Vector3(x=0.0, y=0.0, z=0.3)
            text.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
            text.text = f'{det.class_name}\n{det.confidence:.0%}'
            text.lifetime.sec = 5
            marker_array.markers.append(text)

        self.obj_marker_pub.publish(marker_array)

    def _publish_status_text(self):
        marker_array = MarkerArray()

        # 상태 오버레이 텍스트
        status_text = Marker()
        status_text.header.frame_id = 'map'
        status_text.header.stamp = self.get_clock().now().to_msg()
        status_text.ns = 'status'
        status_text.id = 0
        status_text.type = Marker.TEXT_VIEW_FACING
        status_text.action = Marker.ADD
        status_text.pose.position.x = 0.0
        status_text.pose.position.y = 0.0
        status_text.pose.position.z = 2.0
        status_text.pose.orientation.w = 1.0
        status_text.scale = Vector3(x=0.0, y=0.0, z=0.4)
        status_text.color = ColorRGBA(r=1.0, g=1.0, b=0.0, a=1.0)

        if self.exploration_status:
            s = self.exploration_status
            status_text.text = (
                f'APEX Status\n'
                f'State: {s.state}\n'
                f'Explored: {s.explored_ratio:.1%}\n'
                f'Frontiers: {s.frontiers_count}\n'
                f'Planner: {self.planner_state}\n'
                f'Target Found: {s.target_found}'
            )
        else:
            status_text.text = 'APEX — Initializing...'

        marker_array.markers.append(status_text)
        self.text_marker_pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = VizNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
