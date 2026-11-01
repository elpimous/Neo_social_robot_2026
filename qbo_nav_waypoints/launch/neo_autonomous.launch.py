#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
neo_autonomous.launch.py — Néo se promène seul (waypoints au hasard, regard vivant)
==================================================================================
Auteur : Vincent FOUCAULT

Réutilise toute la pile de test_moving_laser_on_waypoints (lidar, TF laser dynamique,
AMCL, Nav2, collision monitor, boîte noire) SANS le verrou de tête ni la mission
waypoints, et ajoute le nœud neo_autonomous qui pilote la navigation ET la tête.

  ros2 launch qbo_nav_waypoints neo_autonomous.launch.py
  ros2 launch qbo_nav_waypoints neo_autonomous.launch.py duration_min:=60 waypoints:=$HOME/neo_waypoints.yaml
  ros2 launch qbo_nav_waypoints neo_autonomous.launch.py head_travel_deg:=30.0 look_around:=true head_pause_deg:=35.0

Prérequis : qbo_startup lancé (base + tête), puis « 2D Pose Estimate » dans RViz.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg = get_package_share_directory('qbo_nav_waypoints')
    waypoints = LaunchConfiguration('waypoints')
    tilt_level = LaunchConfiguration('tilt_level_rad')
    duration = LaunchConfiguration('duration_min')
    params = LaunchConfiguration('autonomous_params')
    travel_limit = LaunchConfiguration('head_travel_deg')
    look_around = LaunchConfiguration('look_around')
    pause_limit = LaunchConfiguration('head_pause_deg')

    stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg, 'launch', 'test_moving_laser_on_waypoints.launch.py')),
        launch_arguments={'head_lock': 'false', 'mission': 'false',
                          'waypoints': waypoints, 'tilt_level_rad': tilt_level}.items())

    autonomous = Node(
        package='qbo_nav_waypoints', executable='neo_autonomous', name='neo_autonomous',
        output='screen', emulate_tty=True, sigterm_timeout='15',
        parameters=[params, {
            'waypoints_file': waypoints,
            'duration_min': ParameterValue(duration, value_type=float),
            'head.tilt_level_rad': ParameterValue(tilt_level, value_type=float),
            'head.travel_limit_deg': ParameterValue(travel_limit, value_type=float),
            'pause.look_around': ParameterValue(look_around, value_type=bool),
            'pause.limit_deg': ParameterValue(pause_limit, value_type=float),
        }])

    return LaunchDescription([
        DeclareLaunchArgument('waypoints', default_value=os.path.join(pkg, 'config', 'waypoints.yaml')),
        DeclareLaunchArgument('tilt_level_rad', default_value='0.0',
                              description='Tilt qui met le laser horizontal (calibrate_head_level)'),
        DeclareLaunchArgument('duration_min', default_value='0.0', description='0 = illimité'),
        DeclareLaunchArgument('head_travel_deg', default_value='45.0',
                              description='Amplitude max de la tête en roulant (°)'),
        DeclareLaunchArgument('look_around', default_value='false',
                              description='true : la tête regarde autour pendant les pauses'),
        DeclareLaunchArgument('head_pause_deg', default_value='45.0',
                              description='Amplitude max de la tête pendant les pauses (°)'),
        DeclareLaunchArgument('autonomous_params',
                              default_value=os.path.join(pkg, 'config', 'autonomous_params.yaml')),
        stack,
        autonomous,
    ])
