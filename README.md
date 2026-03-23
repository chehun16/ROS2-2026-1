# Autonomous Perception and Exploration

> ROS2 Jazzy | Autonomous Navigation + LiDAR-Camera Fusion + YOLOv8 + Semantic Costmap
> Ubuntu 24.04 LTS

| Component | Version |
|-----------|---------|
| Ubuntu | 24.04 LTS |
| ROS2 | Jazzy Jalisco |
| Python | 3.12 |
| Gazebo | Harmonic (gz sim) |
| SLAM Toolbox | 2.8.x (ros-jazzy-slam-toolbox) |
| Nav2 | 1.3.x (ros-jazzy-navigation2) |
| YOLOv8 (ultralytics) | ≥ 8.0.0 |

---

## Project Overview

An autonomous robot system that explores unknown environments, detects target objects with a camera, estimates precise map coordinates via LiDAR-Camera Fusion, and dynamically replans paths by modifying the Nav2 Costmap in real time based on whether the object should be **approached (APPROACH)** or **avoided (AVOID)**.

```
Sensors:  LiDAR + Camera
Sim:      Gazebo Harmonic + Turtlebot3 waffle_pi
Mapping:  SLAM Toolbox (real-time)
Planning: Nav2 (dynamic replanning with Semantic Costmap)
ML:       YOLOv8 (object detection)
Viz:      RViz2
```

### Node Overview

| No. | Node | Package | Role |
|-----|------|---------|------|
| 1 | `slam_node` | slam | LiDAR → slam_toolbox status monitoring |
| 2 | `perception_node` | perception | Camera + YOLO + **LiDAR-Camera Fusion** → map coordinate estimation |
| 3 | `explorer_node` | explorer | Frontier exploration goal generation |
| 4 | `planner_node` | planner | Integrate detection results → Nav2 dynamic path execution |
| 5 | `viz_node` | viz | RViz2 exploration status overlay |
| 6 | `semantic_costmap_node` | semantic | Detected objects → Nav2 Costmap virtual obstacle injection |
| 7 | `demo_scenario_node` | bringup | Step-by-step status output for demo presentation |

### System Flow

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
                                    Nav2 ObstacleLayer → Costmap real-time update
```

---

## Environment Setup (Windows, WSL2)

> Windows에서는 WSL2를 통해 Ubuntu 24.04 환경을 구성한 뒤, 아래 Ubuntu 설치 과정을 그대로 따라하면 됩니다.

### Step 0-W. Install WSL2 + Ubuntu 24.04

**PowerShell (관리자 권한)** 에서 실행:

```powershell
wsl --install -d Ubuntu-24.04
```

설치 완료 후 재부팅하면 Ubuntu 24.04 터미널이 자동으로 열립니다. username/password 설정 후:

```bash
sudo apt update && sudo apt upgrade -y
```

이후 아래 **Ubuntu 설치 과정 (Step 1~6)** 을 그대로 따라하면 됩니다.

> **주의사항**
> - Gazebo GUI는 WSL2에서 실행 시 [WSLg](https://github.com/microsoft/wslg) 가 필요합니다 (Windows 11 또는 Windows 10 Build 21362+)
> - RViz2, Gazebo 등 GUI 앱은 WSLg가 자동으로 처리합니다 (별도 X서버 불필요)
> - 성능은 네이티브 Ubuntu 대비 다소 낮을 수 있습니다

---

## Environment Setup (Ubuntu 24.04, run in order)

### Step 1. Clone the Repository

```bash
cd ~
git clone https://github.com/chehun16/ROS2-2026-1.git
```

---

### Step 2. Install ROS2 Jazzy

```bash
# Set locale
sudo apt update && sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Add ROS2 apt repository
sudo apt install -y software-properties-common curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS2 Jazzy
sudo apt update
sudo apt install -y ros-jazzy-desktop

# Register environment variable
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

### Step 3. Install Gazebo + Turtlebot3

```bash
# Gazebo Harmonic
sudo apt install -y ros-jazzy-ros-gz

# Turtlebot3
sudo apt install -y \
  ros-jazzy-turtlebot3 \
  ros-jazzy-turtlebot3-simulations

# Register environment variable (waffle_pi has both LiDAR and Camera)
echo "export TURTLEBOT3_MODEL=waffle_pi" >> ~/.bashrc
source ~/.bashrc
```

---

### Step 4. Install SLAM + Nav2

```bash
sudo apt install -y \
  ros-jazzy-slam-toolbox \
  ros-jazzy-navigation2 \
  ros-jazzy-nav2-bringup \
  ros-jazzy-dwb-core \
  ros-jazzy-nav2-dwb-controller
```

---

### Step 5. Install Python and ROS2 Utilities

```bash
# Build tools
sudo apt install -y \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-pip

# ROS2 Python packages
sudo apt install -y \
  ros-jazzy-cv-bridge \
  ros-jazzy-tf2-ros \
  ros-jazzy-tf2-geometry-msgs \
  ros-jazzy-robot-state-publisher \
  ros-jazzy-vision-msgs

# YOLOv8
pip3 install --upgrade pip
pip3 install "ultralytics>=8.0.0"

# Initialize rosdep (first time only)
sudo rosdep init
rosdep update
```

---

### Step 6. Build the Workspace

```bash
cd ~/ROS2-2026-1/apex_ws

# Auto-install dependencies
rosdep install --from-paths src --ignore-src -r -y

# Build
colcon build --symlink-install

# Register environment variable
echo "source ~/ROS2-2026-1/apex_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

## Running

### Standard Launch

```bash
ros2 launch bringup apex.launch.py
```

### Demo Launch (Recommended)

```bash
# demo_scenario_node starts automatically 5 seconds after system startup
# → prints step-by-step exploration logs and detection event banners
ros2 launch bringup demo.launch.py
```

### Individual Node Launch (for debugging)

```bash
# Terminal 1: Gazebo
export TURTLEBOT3_MODEL=waffle_pi
gazebo --verbose ~/ROS2-2026-1/apex_ws/install/bringup/share/bringup/worlds/apex_world.world \
  -s libgazebo_ros_factory.so -s libgazebo_ros_init.so

# Terminal 2: SLAM
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=true \
  slam_params_file:=$HOME/ROS2-2026-1/apex_ws/install/bringup/share/bringup/config/slam_params.yaml

# Terminal 3: Nav2
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true \
  params_file:=$HOME/ROS2-2026-1/apex_ws/install/bringup/share/bringup/config/nav2_params.yaml

# Terminal 4: APEX nodes
ros2 run slam slam_node.py
ros2 run perception perception_node.py
ros2 run explorer explorer_node.py
ros2 run planner planner_node.py
ros2 run viz viz_node.py
ros2 run semantic semantic_costmap_node.py

# Terminal 5: RViz2
ros2 run rviz2 rviz2 -d ~/ROS2-2026-1/apex_ws/install/bringup/share/bringup/config/apex.rviz
```

---

## Package Structure

```
apex_ws/src/
├── msgs/               # Custom messages (DetectedObject, ExplorationStatus)
├── slam/               # Node 1: SLAM monitoring
├── perception/         # Node 2: YOLOv8 + LiDAR-Camera Fusion
│   └── config/yolo_params.yaml
├── explorer/           # Node 3: Frontier Exploration
│   └── config/explorer_params.yaml
├── planner/            # Node 4: Path planning and dynamic replanning
│   └── config/planner_params.yaml
├── viz/                # Node 5: RViz visualization
├── semantic/           # Node 6: Semantic Costmap
└── bringup/
    ├── launch/
    │   ├── apex.launch.py       # Standard launch
    │   └── demo.launch.py       # Demo launch
    ├── config/
    │   ├── slam_params.yaml
    │   ├── nav2_params.yaml
    │   └── apex.rviz
    └── worlds/
        └── apex_world.world     # Gazebo simulation environment (10×10m)
```

---

## Key Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/scan` | sensor_msgs/LaserScan | LiDAR raw data |
| `/camera/image_raw` | sensor_msgs/Image | Camera image |
| `/camera/camera_info` | sensor_msgs/CameraInfo | Camera intrinsic parameters |
| `/map` | nav_msgs/OccupancyGrid | SLAM-generated map |
| `/perception/detections` | msgs/DetectedObject | YOLO + Fusion results |
| `/perception/debug_image` | sensor_msgs/Image | Detection visualization image |
| `/explorer/frontier_goal` | geometry_msgs/PoseStamped | Exploration goal |
| `/explorer/frontiers` | visualization_msgs/MarkerArray | Frontier visualization |
| `/explorer/status` | msgs/ExplorationStatus | Exploration status |
| `/planner/state` | std_msgs/String | Planner state |
| `/semantic/obstacles` | sensor_msgs/PointCloud2 | Virtual obstacles (→ Nav2) |
| `/semantic/markers` | visualization_msgs/MarkerArray | Semantic overlay |
| `/global_costmap/costmap` | nav_msgs/OccupancyGrid | Global costmap |
| `/local_costmap/costmap` | nav_msgs/OccupancyGrid | Local costmap |

---

## Object Classification (Semantic)

| Class | Mode | Behavior |
|-------|------|----------|
| `chair`, `couch`, `bed`, `tv`, `dining table` | **AVOID** | Add virtual obstacle ring to Costmap → reroute path |
| `person`, `bottle`, `cup`, `cell phone`, `laptop` | **APPROACH** | Planner navigates directly to the target |

> **Simulation targets:** `coke_can` × 5 (detected as `bottle`), `cafe_table` × 2 (detected as `dining table`)
> **Confidence threshold:** 0.35 (lowered for Gazebo model textures)

---

## Troubleshooting

**Build failure**
```bash
cd ~/ROS2-2026-1/apex_ws
rm -rf build/ install/ log/
colcon build --symlink-install
```

**Gazebo won't start**
```bash
killall gzserver gzclient
```

**Nav2 fails to come up**
```bash
# Check lifecycle state
ros2 lifecycle list /bt_navigator
# Check logs
ros2 topic echo /bt_navigator/transition_event
```

**LiDAR-Camera Fusion failure (position_valid=False)**
```bash
# Check TF tree — need path: camera_rgb_optical_frame → base_scan
ros2 run tf2_tools view_frames
ros2 topic hz /camera/camera_info
```

**YOLO detection not working**
```bash
# Lower confidence threshold (default 0.35)
# Edit confidence_threshold in perception/config/yolo_params.yaml

# Manually download YOLOv8 model
python3 -c "from ultralytics import YOLO; YOLO('yolov8n.pt')"
```

**Semantic Costmap not updating**
```bash
ros2 topic echo /semantic/obstacles
ros2 topic hz /semantic/obstacles
```
