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
        }])

    return LaunchDescription([
        DeclareLaunchArgument('waypoints', default_value=os.path.join(pkg, 'config', 'waypoints.yaml')),
        DeclareLaunchArgument('tilt_level_rad', default_value='0.0',
                              description='Tilt qui met le laser horizontal (calibrate_head_level)'),
        DeclareLaunchArgument('duration_min', default_value='0.0', description='0 = illimité'),
        DeclareLaunchArgument('autonomous_params',
                              default_value=os.path.join(pkg, 'config', 'autonomous_params.yaml')),
        stack,
        autonomous,
    ])
