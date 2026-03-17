#!/usr/bin/env python3
"""
perception_node.py

Node 2: Perception Node — LiDAR-Camera Fusion
─────────────────────────────────────────────
흐름:
  1. YOLO → bbox 중심 픽셀 (u, v)
  2. camera_info → u를 수평 각도 θ_cam 으로 변환
  3. TF(camera → base_scan) → θ_cam 을 LiDAR 프레임 각도 θ_lidar 로 변환
  4. LaserScan → θ_lidar 에 해당하는 거리 r 조회
  5. LiDAR 프레임 포인트 (r·cosθ, r·sinθ, 0) 생성
  6. TF(base_scan → map) → map 좌표계 위치로 변환
  7. DetectedObject.map_position 에 기록 → planner/costmap 에서 활용

구독:
  /camera/image_raw      (sensor_msgs/Image)
  /camera/camera_info    (sensor_msgs/CameraInfo)
  /scan                  (sensor_msgs/LaserScan)

발행:
  /perception/detections  (msgs/DetectedObject)
  /perception/debug_image (sensor_msgs/Image)
"""

import math
import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan, CameraInfo
from geometry_msgs.msg import Point, PointStamped
from cv_bridge import CvBridge

import tf2_ros
import tf2_geometry_msgs  # PointStamped transform 지원

from msgs.msg import DetectedObject


# ── 상수 ──────────────────────────────────────────────────────────────
# Turtlebot3 waffle_pi LiDAR 유효 범위
LIDAR_RANGE_MIN = 0.12   # m
LIDAR_RANGE_MAX = 3.5    # m

# bbox 중심 픽셀의 수평 방향 탐색 범위 (±픽셀)
# 이 범위 내 LiDAR 빔들의 중앙값을 깊이로 사용 → 노이즈에 강건
BEAM_SEARCH_HALF_DEG = 3.0   # degrees


class PerceptionNode(Node):
    def __init__(self):
        super().__init__('perception_node')

        # ── Parameters ────────────────────────────────────────────────
        self.declare_parameter('model_path', 'yolov8n.pt')
        self.declare_parameter('target_classes', ['person', 'bottle', 'chair'])
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('camera_frame', 'camera_rgb_optical_frame')
        self.declare_parameter('lidar_frame', 'base_scan')

        self.model_path      = self.get_parameter('model_path').value
        self.target_classes  = self.get_parameter('target_classes').value
        self.conf_threshold  = self.get_parameter('confidence_threshold').value
        self.camera_frame    = self.get_parameter('camera_frame').value
        self.lidar_frame     = self.get_parameter('lidar_frame').value

        # ── YOLOv8 ────────────────────────────────────────────────────
        self.model = self._load_model()
        self.bridge = CvBridge()

        # ── TF ────────────────────────────────────────────────────────
        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ── 최신 메시지 캐시 (loose sync) ─────────────────────────────
        self.latest_scan: LaserScan  = None
        self.camera_info: CameraInfo = None

        # ── Subscribers ───────────────────────────────────────────────
        self.create_subscription(Image,      '/camera/image_raw',   self.image_callback,       10)
        self.create_subscription(LaserScan,  '/scan',               self.scan_callback,         10)
        self.create_subscription(CameraInfo, '/camera/camera_info', self.camera_info_callback,  10)

        # ── Publishers ────────────────────────────────────────────────
        self.detection_pub = self.create_publisher(DetectedObject, '/perception/detections', 10)
        self.debug_pub     = self.create_publisher(Image,          '/perception/debug_image', 10)

        self.get_logger().info(
            f'PerceptionNode started | model={self.model_path} | targets={self.target_classes}'
        )

    # ── 콜백 ──────────────────────────────────────────────────────────

    def scan_callback(self, msg: LaserScan):
        self.latest_scan = msg

    def camera_info_callback(self, msg: CameraInfo):
        if self.camera_info is None:
            self.get_logger().info('CameraInfo received')
        self.camera_info = msg

    def image_callback(self, msg: Image):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'cv_bridge error: {e}')
            return

        detections = self._detect_and_fuse(cv_image)

        for det in detections:
            self.detection_pub.publish(det)

        debug_img = self._draw_detections(cv_image, detections)
        self.debug_pub.publish(self.bridge.cv2_to_imgmsg(debug_img, encoding='bgr8'))

    # ── YOLO + Fusion ─────────────────────────────────────────────────

    def _detect_and_fuse(self, image: np.ndarray) -> list:
        if self.model is None:
            return []

        results = self.model(image, conf=self.conf_threshold, verbose=False)
        detections = []

        for result in results:
            for box in result.boxes:
                class_name = self.model.names[int(box.cls[0])]
                if class_name not in self.target_classes:
                    continue

                x1, y1, x2, y2 = box.xyxy[0].tolist()
                cx_px = (x1 + x2) / 2.0
                cy_px = (y1 + y2) / 2.0

                det = DetectedObject()
                det.header.stamp = self.get_clock().now().to_msg()
                det.header.frame_id = 'map'
                det.class_name  = class_name
                det.confidence  = float(box.conf[0])
                det.position    = Point(x=cx_px, y=cy_px, z=0.0)
                det.bbox_x = float(x1)
                det.bbox_y = float(y1)
                det.bbox_w = float(x2 - x1)
                det.bbox_h = float(y2 - y1)

                # ── LiDAR-Camera Fusion ────────────────────────────
                map_pt, dist, valid = self._fuse(cx_px)
                det.map_position   = map_pt
                det.distance       = dist
                det.position_valid = valid

                if valid:
                    self.get_logger().info(
                        f'[FUSION] {class_name} @ map({map_pt.x:.2f}, {map_pt.y:.2f}) '
                        f'dist={dist:.2f}m conf={det.confidence:.2f}'
                    )

                detections.append(det)

        return detections

    def _fuse(self, bbox_cx_px: float) -> tuple[Point, float, bool]:
        """
        bbox 중심 픽셀 x → map 프레임 3D 위치 추정

        Returns:
            (map_point, distance, is_valid)
        """
        FAIL = (Point(), 0.0, False)

        if self.latest_scan is None or self.camera_info is None:
            return FAIL

        # ── Step 1: 픽셀 → 카메라 수평 각도 ──────────────────────────
        fx = self.camera_info.k[0]   # K = [fx, 0, cx, 0, fy, cy, 0, 0, 1]
        cx = self.camera_info.k[2]
        angle_cam = math.atan2(bbox_cx_px - cx, fx)   # radians, 카메라 광학 프레임 기준

        # ── Step 2: TF — 카메라 프레임 → LiDAR 프레임 ─────────────────
        try:
            tf = self.tf_buffer.lookup_transform(
                self.lidar_frame,
                self.camera_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1)
            )
        except Exception:
            return FAIL

        # 카메라 광학 프레임의 Z축 방향 단위벡터를 LiDAR 프레임으로 변환
        # (optical frame: x=right, y=down, z=forward)
        cam_ray = PointStamped()
        cam_ray.header.frame_id = self.camera_frame
        cam_ray.point.x = math.sin(angle_cam)
        cam_ray.point.y = 0.0
        cam_ray.point.z = math.cos(angle_cam)

        try:
            lidar_ray = self.tf_buffer.transform(cam_ray, self.lidar_frame)
        except Exception:
            return FAIL

        angle_lidar = math.atan2(lidar_ray.point.y, lidar_ray.point.x)

        # ── Step 3: LiDAR 빔 인덱스 탐색 + 거리 추출 ─────────────────
        scan = self.latest_scan
        search_rad = math.radians(BEAM_SEARCH_HALF_DEG)

        # 탐색 범위 내 유효 거리값 수집
        distances = []
        num_beams = len(scan.ranges)
        for i in range(num_beams):
            beam_angle = scan.angle_min + i * scan.angle_increment
            if abs(beam_angle - angle_lidar) <= search_rad:
                r = scan.ranges[i]
                if LIDAR_RANGE_MIN < r < LIDAR_RANGE_MAX and math.isfinite(r):
                    distances.append(r)

        if not distances:
            return FAIL

        distance = float(np.median(distances))

        # ── Step 4: LiDAR 프레임 → map 프레임 ─────────────────────────
        pt_in_lidar = PointStamped()
        pt_in_lidar.header.frame_id = self.lidar_frame
        pt_in_lidar.header.stamp    = scan.header.stamp
        pt_in_lidar.point.x = distance * math.cos(angle_lidar)
        pt_in_lidar.point.y = distance * math.sin(angle_lidar)
        pt_in_lidar.point.z = 0.0

        try:
            pt_in_map = self.tf_buffer.transform(
                pt_in_lidar, 'map',
                timeout=rclpy.duration.Duration(seconds=0.1)
            )
        except Exception:
            return FAIL

        map_point = Point(
            x=pt_in_map.point.x,
            y=pt_in_map.point.y,
            z=0.0
        )
        return map_point, distance, True

    # ── 시각화 ────────────────────────────────────────────────────────

    def _draw_detections(self, image: np.ndarray, detections: list) -> np.ndarray:
        img = image.copy()
        for det in detections:
            x, y = int(det.bbox_x), int(det.bbox_y)
            w, h = int(det.bbox_w), int(det.bbox_h)

            color = (0, 255, 0) if det.position_valid else (0, 165, 255)
            cv2.rectangle(img, (x, y), (x + w, y + h), color, 2)

            if det.position_valid:
                label = (
                    f'{det.class_name} {det.confidence:.2f} '
                    f'| {det.distance:.1f}m '
                    f'({det.map_position.x:.1f},{det.map_position.y:.1f})'
                )
            else:
                label = f'{det.class_name} {det.confidence:.2f} | no depth'

            cv2.putText(img, label, (x, y - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

        return img

    # ── 모델 로드 ─────────────────────────────────────────────────────

    def _load_model(self):
        try:
            from ultralytics import YOLO
            model = YOLO(self.model_path)
            self.get_logger().info('YOLOv8 model loaded successfully')
            return model
        except Exception as e:
            self.get_logger().warn(f'YOLOv8 load failed: {e}. Running in mock mode.')
            return None


def main(args=None):
    rclpy.init(args=args)
    node = PerceptionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
