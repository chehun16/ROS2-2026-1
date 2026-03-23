#!/usr/bin/python3
"""
demo_scenario_node.py

발표용 데모 시나리오 노드
─────────────────────────────────────────────────────────────
역할:
  - 시스템 전체 상태를 구독해 터미널에 실시간 요약 출력
  - 탐색 단계별 진행 상황을 발표자가 알기 쉽게 로깅
  - 탐지 이벤트 발생 시 시각적으로 강조 출력

발표 시나리오 흐름:
  [0s]  시스템 시작
  [5s]  이 노드 기동 → "APEX DEMO START" 출력
  [...] explorer가 frontier 탐색 → 맵 생성 중 상태 출력
  [...] perception이 물체 탐지 → "TARGET FOUND!" 강조 출력
  [...] planner가 목표로 이동 → 이동 중 상태 출력
  [...] 목표 도달 → "MISSION COMPLETE" 출력
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from msgs.msg import DetectedObject, ExplorationStatus


class DemoScenarioNode(Node):
    def __init__(self):
        super().__init__('demo_scenario')

        # Subscribers
        self.create_subscription(ExplorationStatus, '/explorer/status',       self.exploration_cb, 10)
        self.create_subscription(DetectedObject,    '/perception/detections', self.detection_cb,   10)
        self.create_subscription(String,            '/planner/state',         self.planner_cb,     10)

        self._last_explorer_state = ''
        self._last_planner_state  = ''
        self._target_announced    = False

        # 1Hz 상태 요약 출력
        self.create_timer(1.0, self.print_status)

        self._exploration_status: ExplorationStatus = None
        self._planner_state = 'WAITING'

        self._banner('APEX DEMO START')
        self.get_logger().info('Demo scenario node running — monitoring system state')

    # ── 콜백 ──────────────────────────────────────────────────

    def exploration_cb(self, msg: ExplorationStatus):
        self._exploration_status = msg
        if msg.state != self._last_explorer_state:
            self._last_explorer_state = msg.state
            if msg.state == 'EXPLORING':
                self.get_logger().info('▶  Phase 1: FRONTIER EXPLORATION started')
            elif msg.state == 'TARGET_FOUND':
                self.get_logger().info('▶  Phase 2: TARGET FOUND — switching to navigation')
            elif msg.state == 'COMPLETED':
                self.get_logger().info('▶  Exploration COMPLETED')

    def detection_cb(self, msg: DetectedObject):
        if not msg.position_valid:
            return
        if not self._target_announced:
            self._target_announced = True
            self._banner(
                f'TARGET DETECTED!\n'
                f'  Class   : {msg.class_name}\n'
                f'  Conf    : {msg.confidence:.0%}\n'
                f'  Distance: {msg.distance:.2f} m\n'
                f'  Map pos : ({msg.map_position.x:.2f}, {msg.map_position.y:.2f})'
            )

    def planner_cb(self, msg: String):
        state = msg.data
        if state != self._last_planner_state:
            self._last_planner_state = state
            if state == 'NAVIGATING_TO_TARGET':
                self.get_logger().info('▶  Phase 3: NAVIGATING TO TARGET')
            elif state == 'COMPLETED':
                self._banner('MISSION COMPLETE\n  Robot reached the target!')

    # ── 주기적 상태 출력 ──────────────────────────────────────

    def print_status(self):
        if self._exploration_status is None:
            return
        s = self._exploration_status
        self.get_logger().info(
            f'[STATUS] state={s.state:<22} '
            f'explored={s.explored_ratio:.1%}  '
            f'frontiers={s.frontiers_count:3d}  '
            f'target_found={s.target_found}'
        )

    # ── 배너 출력 ─────────────────────────────────────────────

    def _banner(self, message: str):
        lines = message.split('\n')
        width = max(len(l) for l in lines) + 4
        border = '=' * width
        self.get_logger().info(border)
        for line in lines:
            self.get_logger().info(f'  {line}')
        self.get_logger().info(border)


def main(args=None):
    rclpy.init(args=args)
    node = DemoScenarioNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
