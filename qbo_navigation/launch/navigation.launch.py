#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node, LifecycleNode


PACKAGE_NAME = "qbo_navigation"

LASER_X = 0.10
LASER_Y = 0.0
LASER_Z = 0.20
LASER_YAW = 3.14159


def generate_launch_description():

    pkg_share = get_package_share_directory(PACKAGE_NAME)

    map_file = os.path.join(pkg_share, "map", "map.yaml")
    params_file = os.path.join(pkg_share, "config", "nav2_params.yaml")

    rviz_config = os.path.join(
        get_package_share_directory('rplidar_ros'),
        'rviz',
        'rplidar_ros.rviz'
    )

    use_sim_time = LaunchConfiguration("use_sim_time", default="false")

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
    )

    # -------------------------------
    # TF lidar -> base
    # -------------------------------

    laser_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="laser_to_base",
        namespace="",
        arguments=[
            "--x", str(LASER_X),
            "--y", str(LASER_Y),
            "--z", str(LASER_Z),
            "--roll", "0", "--pitch", "0",
            "--yaw", str(LASER_YAW),
            "--frame-id", "base_footprint",
            "--child-frame-id", "laser",
        ],
    )

    # -------------------------------
    # RPLIDAR
    # -------------------------------

    rplidar = Node(
        package='rplidar_ros',
        executable='rplidar_node',
        name='rplidar_node',
        #namespace="",
        parameters=[{
            'serial_port': '/dev/ttyRpLidar',
            'serial_baudrate': 115200,
            'frame_id': 'laser',
            'inverted': False,
            'angle_compensate': True,
            'scan_mode': 'Sensitivity',
            'use_sim_time': False,
            'scan_frequency': 10.0
        }],
    )

    # -------------------------------
    # RVIZ
    # -------------------------------

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        namespace="",
        arguments=['-d', rviz_config],
        output='screen'
    )

    # -------------------------------
    # NAV2
    # -------------------------------

    map_server = LifecycleNode(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        namespace="",
        output="screen",
        parameters=[
            {"yaml_filename": map_file},
            {"use_sim_time": use_sim_time},
        ],
    )

    amcl = LifecycleNode(
        package="nav2_amcl",
        executable="amcl",
        name="amcl",
        namespace="",
        output="screen",
        parameters=[params_file, {"use_sim_time": use_sim_time}],
        remappings=[("scan", "/scan")],
    )

    planner_server = LifecycleNode(
        package="nav2_planner",
        executable="planner_server",
        name="planner_server",
        namespace="",
        output="screen",
        parameters=[params_file],
    )

    controller_server = LifecycleNode(
        package="nav2_controller",
        executable="controller_server",
        name="controller_server",
        namespace="",
        output="screen",
        parameters=[params_file],
        remappings=[
            ("cmd_vel", "/qbo_arduqbo/base_ctrl/cmd_vel"),
        ],
    )

    behavior_server = LifecycleNode(
        package="nav2_behaviors",
        executable="behavior_server",
        name="behavior_server",
        namespace="",
        output="screen",
        parameters=[params_file],
    )
    
    bt_navigator = LifecycleNode(
        package="nav2_bt_navigator",
        executable="bt_navigator",
        name="bt_navigator",
        namespace="",
        output="screen",
        parameters=[params_file],
    )

    lifecycle_localization = LifecycleNode(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_localization",
        namespace="",
        output="screen",
        parameters=[{
            "autostart": True,
            "node_names": ["map_server", "amcl"],
        }],
    )

    lifecycle_navigation = LifecycleNode(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_navigation",
        namespace="",
        output="screen",
        parameters=[{
            "autostart": True,
            "node_names": [
                "planner_server",
                "controller_server",
                "bt_navigator",
                "behavior_server"
            ],
        }],
    )

    # -------------------------------
    # ORDRE DE DEMARRAGE
    # -------------------------------

    return LaunchDescription([

        declare_use_sim_time,

        # TF immédiatement
        laser_tf,

        # Lidar après 2s
        TimerAction(
            period=2.0,
            actions=[rplidar]
        ),

        # RViz après 4s
        TimerAction(
            period=4.0,
            actions=[rviz]
        ),

        # Map server après 6s
        TimerAction(
            period=6.0,
            actions=[map_server]
        ),

        # AMCL après 8s
        TimerAction(
            period=8.0,
            actions=[amcl]
        ),

        # Nav2 servers après 10s
        TimerAction(
            period=10.0,
            actions=[
                planner_server,
                controller_server,
                bt_navigator,
                behavior_server
            ]
        ),

        # Lifecycle managers après 12s
        TimerAction(
            period=12.0,
            actions=[
                lifecycle_localization,
                lifecycle_navigation
            ]
        ),

    ])