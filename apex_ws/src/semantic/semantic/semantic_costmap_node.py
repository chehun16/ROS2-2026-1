#!/usr/bin/python3
"""
semantic_costmap_node.py

Node 6: Semantic Costmap Node
─────────────────────────────────────────────────────────────────────
역할:
  탐지된 물체 위치 주변에 가상 장애물(virtual obstacles)을 생성하여
  Nav2 costmap ObstacleLayer에 주입 → 경로 계획 시 해당 영역 회피 또는 접근

핵심 아이디어:
  - DetectedObject.map_position (LiDAR-Camera fusion 결과)를 받아
  - 물체 반경 r 만큼의 원형 가상 장애물 포인트를 PointCloud2로 발행
  - Nav2 ObstacleLayer가 이를 구독 → costmap에 자동 반영
  - 물체별로 "접근(APPROACH)" vs "회피(AVOID)" 모드 지원
    - APPROACH: 탐지 목표 물체 → 주변 cost 낮게 (빈 포인트 발행 안 함)
    - AVOID:    장애물성 물체 → 주변에 가상 장애물 ring 생성

구독:  /perception/detections  (msgs/DetectedObject)
발행:  /semantic/obstacles      (sensor_msgs/PointCloud2)  → Nav2 ObstacleLayer
       /semantic/markers        (visualization_msgs/MarkerArray) → RViz 시각화
"""

import math
import struct
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header, ColorRGBA
from geometry_msgs.msg import Vector3
from visualization_msgs.msg import Marker, MarkerArray

from msgs.msg import DetectedObject


# ── 물체 분류 ──────────────────────────────────────────────────────
# AVOID: costmap에 가상 장애물 ring을 추가해 경로가 멀리 우회하도록 함
# APPROACH: 목표 물체이므로 costmap에 추가하지 않음 (planner가 직접 목표로 이동)
AVOID_CLASSES   = {'chair', 'couch', 'bed', 'dining table', 'tv'}
APPROACH_CLASSES = {'person', 'bottle', 'cup', 'cell phone', 'laptop'}


class SemanticCostmapNode(Node):
    def __init__(self):
        super().__init__('semantic_costmap_node')

        # ── Parameters ────────────────────────────────────────────
        self.declare_parameter('obstacle_radius', 0.6)    # 가상 장애물 ring 반경 (m)
        self.declare_parameter('ring_points', 20)          # ring 포인트 수
        self.declare_parameter('object_timeout', 10.0)    # 물체 기억 시간 (초)
        self.declare_parameter('publish_rate', 2.0)        # Hz

        self.obstacle_radius = self.get_parameter('obstacle_radius').value
        self.ring_points     = self.get_parameter('ring_points').value
        self.object_timeout  = self.get_parameter('object_timeout').value
        publish_rate         = self.get_parameter('publish_rate').value

        # ── 탐지 물체 저장소 ───────────────────────────────────────
        # key: (class_name, grid_x, grid_y)  → 같은 위치 중복 방지
        # value: {'position': Point, 'class': str, 'stamp': float, 'mode': str}
        self.object_map: dict = {}
        self.grid_resolution = 0.3   # m — 이 거리 내 같은 클래스는 동일 객체로 병합

        # ── Subscribers ───────────────────────────────────────────
        self.create_subscription(
            DetectedObject,
            '/perception/detections',
            self.detection_callback,
            10
        )

        # ── Publishers ────────────────────────────────────────────
        self.cloud_pub  = self.create_publisher(PointCloud2,  '/semantic/obstacles', 10)
        self.marker_pub = self.create_publisher(MarkerArray,  '/semantic/markers',   10)

        # ── Timer ─────────────────────────────────────────────────
        self.create_timer(1.0 / publish_rate, self.publish)

        self.get_logger().info(
            f'SemanticCostmapNode started | '
            f'radius={self.obstacle_radius}m | timeout={self.object_timeout}s'
        )

    # ── 콜백 ──────────────────────────────────────────────────────

    def detection_callback(self, msg: DetectedObject):
        if not msg.position_valid:
            return

        mode = 'AVOID' if msg.class_name in AVOID_CLASSES else 'APPROACH'

        # grid key로 중복 병합
        gx = round(msg.map_position.x / self.grid_resolution)
        gy = round(msg.map_position.y / self.grid_resolution)
        key = (msg.class_name, gx, gy)

        self.object_map[key] = {
            'position': msg.map_position,
            'class':    msg.class_name,
            'stamp':    time.time(),
            'mode':     mode,
            'distance': msg.distance,
        }

        self.get_logger().debug(
            f'[SEMANTIC] Stored {msg.class_name} mode={mode} '
            f'@ ({msg.map_position.x:.2f}, {msg.map_position.y:.2f})'
        )

    # ── 발행 ──────────────────────────────────────────────────────

    def publish(self):
        self._expire_objects()

        avoid_objects = [
            obj for obj in self.object_map.values()
            if obj['mode'] == 'AVOID'
        ]

        self._publish_pointcloud(avoid_objects)
        self._publish_markers()

    def _expire_objects(self):
        now = time.time()
        expired = [
            k for k, v in self.object_map.items()
            if now - v['stamp'] > self.object_timeout
        ]
        for k in expired:
            self.get_logger().info(f'[SEMANTIC] Expired: {k[0]}')
            del self.object_map[k]

    def _publish_pointcloud(self, avoid_objects: list):
        """
        AVOID 물체 주변에 원형 가상 장애물 포인트 생성 → PointCloud2 발행
        Nav2 ObstacleLayer가 이를 받아 costmap에 lethal/inflated cost 부여
        """
        points = []
        for obj in avoid_objects:
            ring = self._make_ring(
                obj['position'].x,
                obj['position'].y,
                self.obstacle_radius,
                self.ring_points
            )
            points.extend(ring)

        cloud = self._make_pointcloud2(points)
        self.cloud_pub.publish(cloud)

    def _make_ring(self, cx: float, cy: float, r: float, n: int) -> list:
        """물체 위치 (cx, cy) 주변 반경 r 의 원형 포인트 n개 생성"""
        pts = []
        for i in range(n):
            angle = 2.0 * math.pi * i / n
            pts.append((
                cx + r * math.cos(angle),
                cy + r * math.sin(angle),
                0.0
            ))
        return pts

    def _make_pointcloud2(self, points: list) -> PointCloud2:
        """(x, y, z) 리스트 → sensor_msgs/PointCloud2"""
        header = Header()
        header.stamp    = self.get_clock().now().to_msg()
        header.frame_id = 'map'

        fields = [
            PointField(name='x', offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8,  datatype=PointField.FLOAT32, count=1),
        ]
        point_step = 12   # 3 × float32

        data = bytearray()
        for (x, y, z) in points:
            data += struct.pack('fff', x, y, z)

        cloud = PointCloud2()
        cloud.header      = header
        cloud.height      = 1
        cloud.width       = len(points)
        cloud.fields      = fields
        cloud.is_bigendian = False
        cloud.point_step  = point_step
        cloud.row_step    = point_step * len(points)
        cloud.data        = bytes(data)
        cloud.is_dense    = True
        return cloud

    def _publish_markers(self):
        """RViz 시각화: 물체별 원형 marker + 텍스트 label"""
        marker_array = MarkerArray()
        now = self.get_clock().now().to_msg()

        for i, (key, obj) in enumerate(self.object_map.items()):
            # 원형 디스크 (AVOID=빨강, APPROACH=파랑)
            disk = Marker()
            disk.header.frame_id = 'map'
            disk.header.stamp    = now
            disk.ns   = 'semantic_objects'
            disk.id   = i * 2
            disk.type = Marker.CYLINDER
            disk.action = Marker.ADD
            disk.pose.position   = obj['position']
            disk.pose.position.z = 0.05
            disk.pose.orientation.w = 1.0
            disk.scale = Vector3(
                x=self.obstacle_radius * 2,
                y=self.obstacle_radius * 2,
                z=0.1
            )
            if obj['mode'] == 'AVOID':
                disk.color = ColorRGBA(r=1.0, g=0.2, b=0.2, a=0.4)
            else:
                disk.color = ColorRGBA(r=0.2, g=0.5, b=1.0, a=0.4)
            disk.lifetime.sec = 3
            marker_array.markers.append(disk)

            # 텍스트 label
            text = Marker()
            text.header.frame_id = 'map'
            text.header.stamp    = now
            text.ns   = 'semantic_objects'
            text.id   = i * 2 + 1
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position   = obj['position']
            text.pose.position.z = 0.5
            text.pose.orientation.w = 1.0
            text.scale = Vector3(x=0.0, y=0.0, z=0.25)
            text.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)

            age = time.time() - obj['stamp']
            text.text = (
                f"{obj['class']} [{obj['mode']}]\n"
                f"dist={obj['distance']:.1f}m  age={age:.0f}s"
            )
            text.lifetime.sec = 3
            marker_array.markers.append(text)

        self.marker_pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = SemanticCostmapNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
