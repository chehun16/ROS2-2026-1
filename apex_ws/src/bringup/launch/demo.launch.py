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
    world_file  = os.path.join(bringup_dir, 'worlds', 'apex_world.world')

    # ── Gazebo Harmonic ────────────────────────────────────────
    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', '-r', '--verbose', world_file],
        additional_env={'GZ_SIM_RESOURCE_PATH': os.path.join(bringup_dir, 'worlds')},
        output='screen'
    )

    # /clock 브리지 (sim time 동기화)
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )

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
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', 'turtlebot3_waffle_pi', '-file', urdf_path,
                   '-x', '0.0', '-y', '0.0', '-z', '0.01'],
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
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('nav2_bringup'),
                         'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': os.path.join(bringup_dir, 'config', 'nav2_params.yaml')
        }.items()
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
        gazebo,
        clock_bridge,
        robot_state_publisher,
        spawn_robot,
        slam,
        nav2,
        slam_node,
        perception_node,
        explorer_node,
        planner_node,
        viz_node,
        semantic_node,
        rviz_node,
        demo_node,
    ])
