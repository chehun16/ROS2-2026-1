"""
demo.launch.py

발표용 데모 런치 파일
실행: ros2 launch bringup demo.launch.py

apex.launch.py와의 차이:
  - 데모 시나리오 노드(demo_scenario_node) 추가
    → 시스템 구동 후 자동으로 탐색 시작 + 발표 타이밍에 맞춰 상태 출력
  - Gazebo GUI 없이 실행 가능한 headless 옵션 제공
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    ExecuteProcess,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    headless     = LaunchConfiguration('headless',     default='false')

    declare_use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='true')
    declare_headless     = DeclareLaunchArgument(
        'headless', default_value='false',
        description='Gazebo GUI 없이 실행 (true = 발표장 성능 이슈 대비)'
    )

    set_tb3_model = SetEnvironmentVariable('TURTLEBOT3_MODEL', 'waffle_pi')

    bringup_dir = get_package_share_directory('bringup')
    world_file  = os.path.realpath(
        os.path.join(bringup_dir, 'worlds', 'apex_world.world')
    )

    # ── Gazebo Harmonic ────────────────────────────────────────
    # 외부 플러그인 경로 (diff-drive, joint-state-publisher 등)
    set_gz_plugin_path = SetEnvironmentVariable(
        'GZ_SIM_SYSTEM_PLUGIN_PATH',
        '/opt/ros/jazzy/opt/gz_sim_vendor/lib/'
    )
    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', '-v', '4', world_file],
        additional_env={'GZ_SIM_SYSTEM_PLUGIN_PATH': '/opt/ros/jazzy/opt/gz_sim_vendor/lib/'},
        output='screen'
    )
    # Gazebo 가 완전히 로드된 뒤 시뮬레이션 시작 (physics 충돌 방지)
    unpause_sim = TimerAction(
        period=5.0,
        actions=[
            ExecuteProcess(
                cmd=['gz', 'service', '-s', '/world/apex_world/control',
                     '--reqtype', 'gz.msgs.WorldControl',
                     '--reptype', 'gz.msgs.Boolean',
                     '--timeout', '2000',
                     '--req', 'pause: false'],
                output='screen'
            )
        ]
    )

    # ROS2 ↔ Gazebo Harmonic 브리지
    # 로봇은 world 파일에 내장 (DiffDrive 플러그인이 odom/tf 직접 publish)
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
        ],
        output='screen'
    )

    # robot_state_publisher: URDF 기반 정적 TF 트리
    # 로봇 스폰은 world 파일에서 처리
    urdf_path = os.path.join(
        get_package_share_directory('turtlebot3_description'),
        'urdf', 'turtlebot3_waffle_pi.urdf'
    )
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': open(urdf_path).read()
        }],
        output='screen'
    )

    # ── SLAM + Nav2 ────────────────────────────────────────────
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('slam_toolbox'),
                         'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'slam_params_file': os.path.join(bringup_dir, 'config', 'slam_params.yaml')
        }.items()
    )
    nav2_params = os.path.join(bringup_dir, 'config', 'nav2_params.yaml')

    nav2_controller   = Node(package='nav2_controller',       executable='controller_server',  output='screen', parameters=[nav2_params])
    nav2_smoother     = Node(package='nav2_smoother',          executable='smoother_server',    output='screen', parameters=[nav2_params])
    nav2_planner      = Node(package='nav2_planner',           executable='planner_server',     output='screen', parameters=[nav2_params])
    nav2_behaviors    = Node(package='nav2_behaviors',         executable='behavior_server',    output='screen', parameters=[nav2_params])
    nav2_bt           = Node(package='nav2_bt_navigator',      executable='bt_navigator',       output='screen', parameters=[nav2_params])
    nav2_waypoint     = Node(package='nav2_waypoint_follower', executable='waypoint_follower',  output='screen', parameters=[nav2_params])
    nav2_vel_smoother = Node(package='nav2_velocity_smoother', executable='velocity_smoother',  output='screen', parameters=[nav2_params])
    nav2_lifecycle    = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': [
                'controller_server', 'smoother_server', 'planner_server',
                'behavior_server', 'bt_navigator', 'waypoint_follower',
                'velocity_smoother'
            ]
        }]
    )

    # ── APEX 노드들 ────────────────────────────────────────────
    slam_node = Node(
        package='slam', executable='slam_node.py', name='slam_node',
        parameters=[{'use_sim_time': use_sim_time}], output='screen'
    )
    perception_node = Node(
        package='perception', executable='perception_node.py', name='perception_node',
        parameters=[
            os.path.join(get_package_share_directory('perception'), 'config', 'yolo_params.yaml'),
            {'use_sim_time': use_sim_time}
        ],
        output='screen'
    )
    explorer_node = Node(
        package='explorer', executable='explorer_node.py', name='explorer_node',
        parameters=[
            os.path.join(get_package_share_directory('explorer'), 'config', 'explorer_params.yaml'),
            {'use_sim_time': use_sim_time}
        ],
        output='screen'
    )
    planner_node = Node(
        package='planner', executable='planner_node.py', name='planner_node',
        parameters=[
            os.path.join(get_package_share_directory('planner'), 'config', 'planner_params.yaml'),
            {'use_sim_time': use_sim_time}
        ],
        output='screen'
    )
    viz_node = Node(
        package='viz', executable='viz_node.py', name='viz_node',
        parameters=[{'use_sim_time': use_sim_time}], output='screen'
    )
    semantic_node = Node(
        package='semantic', executable='semantic_costmap_node.py', name='semantic_costmap_node',
        parameters=[{'use_sim_time': use_sim_time}], output='screen'
    )

    # ── Gazebo 센서 frame_id 보정용 static TF ──────────────────
    scan_frame_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='scan_frame_bridge',
        arguments=['--frame-id', 'base_scan',
                   '--child-frame-id', 'turtlebot3_waffle_pi/base_scan/lidar'],
        output='screen'
    )
    camera_frame_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_frame_bridge',
        arguments=['--frame-id', 'camera_rgb_optical_frame',
                   '--child-frame-id', 'turtlebot3_waffle_pi/camera_rgb_frame/camera'],
        output='screen'
    )

    # ── RViz2 ──────────────────────────────────────────────────
    rviz_node = Node(
        package='rviz2', executable='rviz2', name='rviz2',
        arguments=['-d', os.path.join(bringup_dir, 'config', 'apex.rviz')],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # ── 데모 시나리오 노드 (5초 후 시작) ──────────────────────
    # 시스템이 올라온 뒤 자동으로 탐색 시작 알림 + 상태 모니터링
    demo_node = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='bringup',
                executable='demo_scenario_node.py',
                name='demo_scenario',
                parameters=[{'use_sim_time': use_sim_time}],
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_headless,
        set_tb3_model,
        set_gz_plugin_path,
        gazebo,
        clock_bridge,
        robot_state_publisher,
        slam,
        nav2_controller,
        nav2_smoother,
        nav2_planner,
        nav2_behaviors,
        nav2_bt,
        nav2_waypoint,
        nav2_vel_smoother,
        TimerAction(period=7.0, actions=[nav2_lifecycle]),
        slam_node,
        perception_node,
        explorer_node,
        planner_node,
        viz_node,
        semantic_node,
        rviz_node,
        scan_frame_tf,
        camera_frame_tf,
        unpause_sim,
        demo_node,
    ])
