"""
apex.launch.py

전체 시스템 런치 파일
실행: ros2 launch bringup apex.launch.py
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    ExecuteProcess,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # ── 인자 선언 ──────────────────────────────────────────────
    use_sim_time     = LaunchConfiguration('use_sim_time',     default='true')
    turtlebot3_model = LaunchConfiguration('turtlebot3_model', default='waffle_pi')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation (Gazebo) clock'
    )
    declare_model = DeclareLaunchArgument(
        'turtlebot3_model', default_value='waffle_pi',
        description='Turtlebot3 model — waffle_pi has LiDAR + Camera'
    )

    set_tb3_model = SetEnvironmentVariable('TURTLEBOT3_MODEL', turtlebot3_model)

    # ── 패키지 경로 ────────────────────────────────────────────
    bringup_dir      = get_package_share_directory('bringup')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    world_file       = os.path.join(bringup_dir, 'worlds', 'apex_world.world')

    # ── Gazebo Harmonic: 커스텀 월드로 실행 ───────────────────
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

    # Turtlebot3 robot_state_publisher (URDF → TF)
    urdf_path = os.path.join(
        get_package_share_directory('turtlebot3_description'),
        'urdf', 'turtlebot3_waffle_pi.urdf'
    )
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': open(urdf_path).read()
        }],
        output='screen'
    )

    # Turtlebot3 Gazebo 스폰 (Gazebo Harmonic)
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'turtlebot3_waffle_pi',
            '-file', urdf_path,
            '-x', '0.0', '-y', '0.0', '-z', '0.01'
        ],
        output='screen'
    )

    # ── SLAM Toolbox ───────────────────────────────────────────
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('slam_toolbox'),
                'launch', 'online_async_launch.py'
            )
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'slam_params_file': os.path.join(bringup_dir, 'config', 'slam_params.yaml')
        }.items()
    )

    # ── Nav2 ───────────────────────────────────────────────────
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': os.path.join(bringup_dir, 'config', 'nav2_params.yaml')
        }.items()
    )

    # ── APEX Nodes ─────────────────────────────────────────────
    slam_node = Node(
        package='slam',
        executable='slam_node.py',
        name='slam_node',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    perception_node = Node(
        package='perception',
        executable='perception_node.py',
        name='perception_node',
        parameters=[
            os.path.join(
                get_package_share_directory('perception'),
                'config', 'yolo_params.yaml'
            ),
            {'use_sim_time': use_sim_time}
        ],
        output='screen'
    )

    explorer_node = Node(
        package='explorer',
        executable='explorer_node.py',
        name='explorer_node',
        parameters=[
            os.path.join(
                get_package_share_directory('explorer'),
                'config', 'explorer_params.yaml'
            ),
            {'use_sim_time': use_sim_time}
        ],
        output='screen'
    )

    planner_node = Node(
        package='planner',
        executable='planner_node.py',
        name='planner_node',
        parameters=[
            os.path.join(
                get_package_share_directory('planner'),
                'config', 'planner_params.yaml'
            ),
            {'use_sim_time': use_sim_time}
        ],
        output='screen'
    )

    viz_node = Node(
        package='viz',
        executable='viz_node.py',
        name='viz_node',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    semantic_costmap_node = Node(
        package='semantic',
        executable='semantic_costmap_node.py',
        name='semantic_costmap_node',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # ── RViz2 ──────────────────────────────────────────────────
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(bringup_dir, 'config', 'apex.rviz')],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_model,
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
        semantic_costmap_node,
        rviz_node,
    ])
