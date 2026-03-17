# APEX — Autonomous Perception and EXploration

> ROS2 Humble 기반 자율 탐색 + LiDAR-Camera Fusion + YOLOv8 + Semantic Costmap
> Ubuntu 22.04 LTS

---

## 프로젝트 개요

미지 환경을 자율 탐색하며 카메라로 목표 물체를 탐지하면,
LiDAR-Camera Fusion으로 정확한 map 좌표를 추정하고,
물체 종류에 따라 **접근(APPROACH)** 또는 **회피(AVOID)** 를 결정해
Nav2 Costmap을 실시간으로 수정하며 경로를 재계획하는 자율 로봇 시스템.

```
센서: LiDAR + Camera
시뮬: Gazebo Classic + Turtlebot3 waffle_pi
맵핑: SLAM Toolbox (실시간)
경로: Nav2 (Semantic Costmap 기반 동적 재계획)
ML:   YOLOv8 (물체 탐지)
시각: RViz2
```

### 노드 구성

| No. | 노드 | 패키지 | 역할 |
|-----|------|--------|------|
| 1 | `slam_node` | slam | LiDAR → slam_toolbox 상태 모니터링 |
| 2 | `perception_node` | perception | Camera + YOLO + **LiDAR-Camera Fusion** → map 좌표 추정 |
| 3 | `explorer_node` | explorer | Frontier Exploration 목표 생성 |
| 4 | `planner_node` | planner | 탐지 결과 통합 → Nav2 동적 경로 실행 |
| 5 | `viz_node` | viz | RViz2 탐색 현황 오버레이 |
| 6 | `semantic_costmap_node` | semantic | 탐지 물체 → Nav2 Costmap 가상 장애물 주입 |
| 7 | `demo_scenario_node` | bringup | 발표용 단계별 상태 출력 |

### 시스템 흐름

```
LiDAR ──▶ slam_toolbox ──▶ /map ──▶ explorer_node ──▶ frontier_goal ──▶ planner_node
                                                                               ▲
Camera ──▶ YOLOv8 + LiDAR Fusion ──▶ /perception/detections ─────────────────┤
                                              │                                │
                                              ▼                         Nav2 NavigateToPose
                                    semantic_costmap_node
                                              │
                                    /semantic/obstacles (PointCloud2)
                                              │
                                    Nav2 ObstacleLayer → Costmap 실시간 갱신
```

---

## 환경 설정 (Ubuntu 22.04 기준, 순서대로 실행)

### Step 1. 저장소 클론

```bash
cd ~
git clone https://github.com/chehun16/ROS2-2026-1.git
```

---

### Step 2. ROS2 Humble 설치

```bash
# locale 설정
sudo apt update && sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# ROS2 apt 저장소 추가
sudo apt install -y software-properties-common curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# ROS2 Humble 설치
sudo apt update
sudo apt install -y ros-humble-desktop

# 환경 변수 등록
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

### Step 3. Gazebo + Turtlebot3 설치

```bash
# Gazebo Classic 11
sudo apt install -y gazebo libgazebo-dev

# Gazebo-ROS 연동 플러그인
sudo apt install -y \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-gazebo-ros2-control

# Turtlebot3
sudo apt install -y \
  ros-humble-turtlebot3 \
  ros-humble-turtlebot3-simulations \
  ros-humble-turtlebot3-gazebo

# 환경 변수 등록 (waffle_pi = LiDAR + Camera 둘 다 있음)
echo "export TURTLEBOT3_MODEL=waffle_pi" >> ~/.bashrc
source ~/.bashrc
```

---

### Step 4. SLAM + Nav2 설치

```bash
sudo apt install -y \
  ros-humble-slam-toolbox \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-dwb-core \
  ros-humble-nav2-dwb-controller
```

---

### Step 5. Python 및 ROS2 유틸 설치

```bash
# 빌드 도구
sudo apt install -y \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-pip

# ROS2 Python 패키지
sudo apt install -y \
  ros-humble-cv-bridge \
  ros-humble-tf2-ros \
  ros-humble-tf2-geometry-msgs \
  ros-humble-robot-state-publisher \
  ros-humble-vision-msgs

# YOLOv8
pip3 install --upgrade pip
pip3 install ultralytics

# rosdep 초기화 (최초 1회)
sudo rosdep init
rosdep update
```

---

### Step 6. 워크스페이스 빌드

```bash
cd ~/ROS2-2026-1/apex_ws

# 의존성 자동 설치
rosdep install --from-paths src --ignore-src -r -y

# 빌드
colcon build --symlink-install

# 환경 변수 등록
echo "source ~/ROS2-2026-1/apex_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

## 실행

### 일반 실행

```bash
ros2 launch bringup apex.launch.py
```

### 발표용 실행 (권장)

```bash
# 시스템 기동 5초 후 demo_scenario_node 자동 시작
# → 탐색 단계별 로그, 탐지 이벤트 배너 출력
ros2 launch bringup demo.launch.py
```

### 개별 노드 실행 (디버깅용)

```bash
# 터미널 1: Gazebo
export TURTLEBOT3_MODEL=waffle_pi
gazebo --verbose ~/ROS2-2026-1/apex_ws/install/bringup/share/bringup/worlds/apex_world.world \
  -s libgazebo_ros_factory.so -s libgazebo_ros_init.so

# 터미널 2: SLAM
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=true \
  slam_params_file:=$HOME/ROS2-2026-1/apex_ws/install/bringup/share/bringup/config/slam_params.yaml

# 터미널 3: Nav2
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true \
  params_file:=$HOME/ROS2-2026-1/apex_ws/install/bringup/share/bringup/config/nav2_params.yaml

# 터미널 4: APEX 노드들
ros2 run slam slam_node.py
ros2 run perception perception_node.py
ros2 run explorer explorer_node.py
ros2 run planner planner_node.py
ros2 run viz viz_node.py
ros2 run semantic semantic_costmap_node.py

# 터미널 5: RViz2
ros2 run rviz2 rviz2 -d ~/ROS2-2026-1/apex_ws/install/bringup/share/bringup/config/apex.rviz
```

---

## 패키지 구조

```
apex_ws/src/
├── msgs/               # 커스텀 메시지 (DetectedObject, ExplorationStatus)
├── slam/               # Node 1: SLAM 모니터링
├── perception/         # Node 2: YOLOv8 + LiDAR-Camera Fusion
│   └── config/yolo_params.yaml
├── explorer/           # Node 3: Frontier Exploration
│   └── config/explorer_params.yaml
├── planner/            # Node 4: 경로 계획 및 동적 재계획
│   └── config/planner_params.yaml
├── viz/                # Node 5: RViz 시각화
├── semantic/           # Node 6: Semantic Costmap
└── bringup/
    ├── launch/
    │   ├── apex.launch.py       # 일반 실행
    │   └── demo.launch.py       # 발표용 실행
    ├── config/
    │   ├── slam_params.yaml
    │   ├── nav2_params.yaml
    │   └── apex.rviz
    └── worlds/
        └── apex_world.world     # Gazebo 시뮬 환경
```

---

## 주요 토픽

| 토픽 | 타입 | 설명 |
|------|------|------|
| `/scan` | sensor_msgs/LaserScan | LiDAR 원시 데이터 |
| `/camera/image_raw` | sensor_msgs/Image | 카메라 이미지 |
| `/camera/camera_info` | sensor_msgs/CameraInfo | 카메라 내부 파라미터 |
| `/map` | nav_msgs/OccupancyGrid | SLAM 생성 맵 |
| `/perception/detections` | msgs/DetectedObject | YOLO + Fusion 결과 |
| `/perception/debug_image` | sensor_msgs/Image | 탐지 시각화 이미지 |
| `/explorer/frontier_goal` | geometry_msgs/PoseStamped | 탐색 목표 |
| `/explorer/frontiers` | visualization_msgs/MarkerArray | frontier 시각화 |
| `/explorer/status` | msgs/ExplorationStatus | 탐색 상태 |
| `/planner/state` | std_msgs/String | planner 상태 |
| `/semantic/obstacles` | sensor_msgs/PointCloud2 | 가상 장애물 (→ Nav2) |
| `/semantic/markers` | visualization_msgs/MarkerArray | 시맨틱 오버레이 |
| `/global_costmap/costmap` | nav_msgs/OccupancyGrid | 전역 비용 맵 |
| `/local_costmap/costmap` | nav_msgs/OccupancyGrid | 지역 비용 맵 |

---

## 물체 분류 (Semantic)

| 클래스 | 모드 | 동작 |
|--------|------|------|
| `chair`, `couch`, `bed`, `tv`, `dining table` | **AVOID** | Costmap 가상 장애물 ring 추가 → 경로 우회 |
| `person`, `bottle`, `cup`, `cell phone`, `laptop` | **APPROACH** | Planner가 직접 목표로 이동 |

---

## 트러블슈팅

**build 실패 시**
```bash
cd ~/ROS2-2026-1/apex_ws
rm -rf build/ install/ log/
colcon build --symlink-install
```

**Gazebo가 안 켜질 때**
```bash
killall gzserver gzclient
```

**Nav2가 올라오지 않을 때**
```bash
# lifecycle 상태 확인
ros2 lifecycle list /bt_navigator
# 로그 확인
ros2 topic echo /bt_navigator/transition_event
```

**LiDAR-Camera Fusion 실패 (position_valid=False)**
```bash
# TF 트리 확인 — camera_rgb_optical_frame → base_scan 경로 필요
ros2 run tf2_tools view_frames
ros2 topic hz /camera/camera_info
```

**YOLO 탐지가 안 될 때**
```bash
# 신뢰도 임계값 낮추기 (기본 0.5)
# perception/config/yolo_params.yaml 에서 confidence_threshold 수정

# YOLOv8 모델 수동 다운로드
python3 -c "from ultralytics import YOLO; YOLO('yolov8n.pt')"
```

**Semantic Costmap 미반영 시**
```bash
ros2 topic echo /semantic/obstacles
ros2 topic hz /semantic/obstacles
```
