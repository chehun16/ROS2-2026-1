#!/usr/bin/python3
"""
explorer_node.py

Node 3: Explorer Node
- Subscribes to /map (OccupancyGrid)
- Frontier Exploration 알고리즘으로 미탐색 영역 계산 (numpy 벡터화)
- 로봇 현재 위치(TF)를 기반으로 가장 가까운 frontier 선택
- /explorer/frontier_goal (PoseStamped) publish → planner_node가 수신
- /explorer/frontiers (MarkerArray) publish → RViz 시각화
- /explorer/status (ExplorationStatus) publish
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np
import tf2_ros

from msgs.msg import DetectedObject, ExplorationStatus


class ExplorerNode(Node):
    def __init__(self):
        super().__init__('explorer_node')

        self.declare_parameter('exploration_rate', 1.0)   # Hz (맵 크기 고려해 낮게)
        self.declare_parameter('min_frontier_distance', 0.5)  # 너무 가까운 frontier 무시 (m)

        exploration_rate        = self.get_parameter('exploration_rate').value
        self.min_frontier_dist  = self.get_parameter('min_frontier_distance').value

        # TF — 로봇 현재 위치 조회용
        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Subscribers
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, 10
        )
        self.detection_sub = self.create_subscription(
            DetectedObject, '/perception/detections', self.detection_callback, 10
        )

        # Publishers
        self.goal_pub   = self.create_publisher(PoseStamped,  '/explorer/frontier_goal', 10)
        self.marker_pub = self.create_publisher(MarkerArray,  '/explorer/frontiers',     10)
        self.status_pub = self.create_publisher(ExplorationStatus, '/explorer/status',   10)

        # Timer
        self.timer = self.create_timer(1.0 / exploration_rate, self.explore)

        self.map_data: OccupancyGrid = None
        self.target_found   = False
        self.target_position = Point()
        self.state = 'IDLE'

        self.get_logger().info('ExplorerNode started')

    # ── 콜백 ──────────────────────────────────────────────────────────

    def map_callback(self, msg: OccupancyGrid):
        self.map_data = msg
        if self.state == 'IDLE':
            self.state = 'EXPLORING'

    def detection_callback(self, msg: DetectedObject):
        if self.target_found:
            return
        if not msg.position_valid:
            self.get_logger().warn(
                f'Detected {msg.class_name} but no valid 3D position — skipping'
            )
            return
        self.get_logger().info(
            f'Target detected: {msg.class_name} ({msg.confidence:.2f}) '
            f'@ map({msg.map_position.x:.2f}, {msg.map_position.y:.2f}) '
            f'dist={msg.distance:.2f}m'
        )
        self.target_found    = True
        self.target_position = msg.map_position
        self.state = 'TARGET_FOUND'

    # ── 탐색 루프 ─────────────────────────────────────────────────────

    def explore(self):
        if self.map_data is None:
            return

        if self.target_found:
            self._publish_status(frontiers=[])
            return

        frontiers = self._find_frontiers(self.map_data)

        if not frontiers:
            self.state = 'COMPLETED'
            self._publish_status(frontiers=[])
            return

        best = self._select_nearest_frontier(frontiers)
        self._publish_goal(best)
        self._publish_markers(frontiers)
        self._publish_status(frontiers)

    # ── Frontier 탐지 (numpy 벡터화) ─────────────────────────────────

    def _find_frontiers(self, map_msg: OccupancyGrid) -> list:
        """
        Frontier: free(0) 셀에 인접한 unknown(-1) 셀
        numpy 롤링 연산으로 O(w*h) Python 루프 없이 계산
        """
        w = map_msg.info.width
        h = map_msg.info.height
        data = np.array(map_msg.data, dtype=np.int8).reshape((h, w))

        free    = (data == 0)
        unknown = (data == -1)

        # 4방향으로 unknown 셀이 인접한 free 셀 → frontier (free 공간 안에서 선택)
        unknown_adj = (
            np.roll(unknown,  1, axis=0) |
            np.roll(unknown, -1, axis=0) |
            np.roll(unknown,  1, axis=1) |
            np.roll(unknown, -1, axis=1)
        )
        frontier_mask = free & unknown_adj

        # 테두리 제거 (roll 아티팩트)
        frontier_mask[0, :]  = False
        frontier_mask[-1, :] = False
        frontier_mask[:, 0]  = False
        frontier_mask[:, -1] = False

        ys, xs = np.where(frontier_mask)
        if len(xs) == 0:
            return []

        res = map_msg.info.resolution
        ox  = map_msg.info.origin.position.x
        oy  = map_msg.info.origin.position.y

        frontiers = [
            (ox + (x + 0.5) * res, oy + (y + 0.5) * res)
            for x, y in zip(xs.tolist(), ys.tolist())
        ]
        return frontiers

    # ── Frontier 선택: 로봇과 가장 가까운 frontier ───────────────────

    def _select_nearest_frontier(self, frontiers: list) -> tuple:
        rx, ry = self._get_robot_position()

        # 너무 가까운 frontier는 제외 (이미 탐색한 영역 근처)
        filtered = [
            (fx, fy) for fx, fy in frontiers
            if (fx - rx) ** 2 + (fy - ry) ** 2 > self.min_frontier_dist ** 2
        ]
        if not filtered:
            filtered = frontiers   # 전부 가까우면 그냥 전체에서 선택

        dists = np.array([(fx - rx) ** 2 + (fy - ry) ** 2 for fx, fy in filtered])
        fx, fy = filtered[int(np.argmin(dists))]

        # frontier 경계에서 로봇 방향으로 0.4m 이동 → costmap inflation 영역 밖 보장
        dx, dy = rx - fx, ry - fy
        dist = float(np.sqrt(dx ** 2 + dy ** 2))
        if dist > 0.01:
            offset = min(0.4, dist - 0.6)   # 로봇으로부터 최소 0.6m 유지
            if offset > 0:
                fx = fx + offset * dx / dist
                fy = fy + offset * dy / dist

        return (fx, fy)

    def _get_robot_position(self) -> tuple:
        """TF에서 map 기준 로봇 위치를 가져옴. 실패 시 원점 반환."""
        try:
            tf = self.tf_buffer.lookup_transform(
                'map', 'base_footprint',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1)
            )
            return tf.transform.translation.x, tf.transform.translation.y
        except Exception:
            # base_footprint 없으면 base_link 시도
            try:
                tf = self.tf_buffer.lookup_transform(
                    'map', 'base_link',
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.1)
                )
                return tf.transform.translation.x, tf.transform.translation.y
            except Exception:
                return 0.0, 0.0

    # ── 발행 ──────────────────────────────────────────────────────────

    def _publish_goal(self, frontier: tuple):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp    = self.get_clock().now().to_msg()
        goal.pose.position.x = frontier[0]
        goal.pose.position.y = frontier[1]
        goal.pose.orientation.w = 1.0
        self.goal_pub.publish(goal)

    def _publish_markers(self, frontiers: list):
        marker_array = MarkerArray()
        stamp = self.get_clock().now().to_msg()
        for i, (fx, fy) in enumerate(frontiers[:100]):
            m = Marker()
            m.header.frame_id = 'map'
            m.header.stamp    = stamp
            m.ns     = 'frontiers'
            m.id     = i
            m.type   = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = fx
            m.pose.position.y = fy
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = 0.15
            m.color.r = 1.0
            m.color.g = 0.5
            m.color.a = 0.7
            m.lifetime.sec = 2
            marker_array.markers.append(m)
        self.marker_pub.publish(marker_array)

    def _publish_status(self, frontiers: list):
        status = ExplorationStatus()
        status.header.stamp    = self.get_clock().now().to_msg()
        status.header.frame_id = 'map'
        status.state           = self.state
        status.frontiers_count = len(frontiers)
        status.target_found    = self.target_found
        if self.target_found:
            status.target_position = self.target_position

        if self.map_data is not None:
            data = self.map_data.data
            total = len(data)
            if total > 0:
                known = int(np.sum(np.array(data, dtype=np.int8) >= 0))
                status.explored_ratio = known / total

        self.status_pub.publish(status)


def main(args=None):
    rclpy.init(args=args)
    node = ExplorerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
