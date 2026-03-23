#!/usr/bin/python3
"""
planner_node.py

Node 4: Planner Node
- Subscribes to /explorer/frontier_goal  (탐색 목표)
- Subscribes to /perception/detections   (물체 탐지 → 최우선 목표)
- Subscribes to /explorer/status         (탐색 상태)
- Nav2 NavigateToPose action으로 목표 전송
- 탐지 발생 시 즉시 현재 goal을 취소하고 탐지 위치로 재계획

수정사항:
- Nav2 대기를 비동기 타이머 방식으로 변경 (blocking 제거)
- 탐지 쿨다운으로 중복 goal 방지
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import String

from msgs.msg import DetectedObject, ExplorationStatus


class PlannerNode(Node):
    def __init__(self):
        super().__init__('planner_node')

        self.declare_parameter('target_cooldown_sec', 5.0)
        self._target_cooldown = self.get_parameter('target_cooldown_sec').value

        # Nav2 Action Client
        self._nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Subscribers
        self.create_subscription(PoseStamped,        '/explorer/frontier_goal', self.frontier_callback,  10)
        self.create_subscription(DetectedObject,     '/perception/detections',  self.detection_callback, 10)
        self.create_subscription(ExplorationStatus,  '/explorer/status',        self.status_callback,    10)

        # Publisher
        self.state_pub = self.create_publisher(String, '/planner/state', 10)

        # State
        self.goal_handle          = None
        self.state                = 'WAITING'
        self._nav_ready           = False
        self._last_target_time    = None   # 마지막 target goal 전송 시각
        self._pending_frontier    = None   # Nav2 준비 전에 들어온 frontier 캐시

        # Nav2 비동기 대기 타이머 (1Hz 폴링)
        self._nav_check_timer = self.create_timer(1.0, self._check_nav2_ready)

        self.get_logger().info('PlannerNode started — waiting for Nav2...')

    # ── Nav2 준비 확인 (비동기) ────────────────────────────────────────

    def _check_nav2_ready(self):
        if self._nav_client.server_is_ready():
            self.get_logger().info('Nav2 action server ready!')
            self._nav_ready = True
            self._nav_check_timer.cancel()
            self.state = 'READY'
            self._publish_state()
            # 대기 중이던 frontier goal이 있으면 전송
            if self._pending_frontier is not None:
                self._send_goal(self._pending_frontier, 'FRONTIER')
                self._pending_frontier = None
        else:
            self.get_logger().info('Waiting for Nav2 NavigateToPose server...')

    # ── 콜백 ──────────────────────────────────────────────────────────

    def frontier_callback(self, msg: PoseStamped):
        if self.state in ('NAVIGATING_TO_TARGET', 'COMPLETED'):
            return
        if not self._nav_ready:
            self._pending_frontier = msg   # Nav2 준비될 때까지 캐시
            return
        # Nav2가 이미 이동 중이면 새 frontier goal은 무시 (도달 후 explorer가 새로 줌)
        if self.state == 'NAVIGATING':
            return
        self._send_goal(msg, 'FRONTIER')

    def detection_callback(self, msg: DetectedObject):
        if self.state == 'COMPLETED' or not self._nav_ready:
            return
        if not msg.position_valid:
            return

        # 쿨다운: 같은 목표를 반복해서 보내지 않도록
        now = self.get_clock().now()
        if self._last_target_time is not None:
            elapsed = (now - self._last_target_time).nanoseconds / 1e9
            if elapsed < self._target_cooldown:
                return

        self.get_logger().info(
            f'[PLANNER] Target! class={msg.class_name} '
            f'map=({msg.map_position.x:.2f}, {msg.map_position.y:.2f}) '
            f'dist={msg.distance:.2f}m'
        )

        # 현재 goal 취소
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
            self.goal_handle = None

        goal_pose = PoseStamped()
        goal_pose.header.frame_id   = 'map'
        goal_pose.header.stamp      = self.get_clock().now().to_msg()
        goal_pose.pose.position     = msg.map_position
        goal_pose.pose.orientation.w = 1.0

        self._last_target_time = now
        self.state = 'NAVIGATING_TO_TARGET'
        self._send_goal(goal_pose, 'TARGET')

    def status_callback(self, msg: ExplorationStatus):
        if msg.state == 'COMPLETED' and self.state not in ('NAVIGATING_TO_TARGET', 'COMPLETED'):
            self.get_logger().info('Exploration completed — no target found.')
            self.state = 'COMPLETED'
            self._publish_state()

    # ── Nav2 Goal 전송 ────────────────────────────────────────────────

    def _send_goal(self, pose: PoseStamped, priority: str):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose

        self.get_logger().info(
            f'[PLANNER] Sending {priority} goal: '
            f'({pose.pose.position.x:.2f}, {pose.pose.position.y:.2f})'
        )

        if priority == 'FRONTIER':
            self.state = 'NAVIGATING'

        send_future = self._nav_client.send_goal_async(
            goal_msg,
            feedback_callback=self._feedback_callback
        )
        send_future.add_done_callback(self._goal_response_callback)
        self._publish_state()

    def _goal_response_callback(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().warn('Goal rejected by Nav2')
            self.state = 'READY'
            self._publish_state()
            return
        self.goal_handle = handle
        handle.get_result_async().add_done_callback(self._result_callback)

    def _feedback_callback(self, feedback_msg):
        pass

    def _result_callback(self, future):
        if self.state == 'NAVIGATING_TO_TARGET':
            self.get_logger().info('Target reached! Mission complete.')
            self.state = 'COMPLETED'
        else:
            self.state = 'READY'
        self.goal_handle = None
        self._publish_state()

    def _publish_state(self):
        msg = String()
        msg.data = self.state
        self.state_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
