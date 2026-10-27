#!/usr/bin/env python3
"""
listen_and_talk.launch.py — lance listen_node et talk_node dans DEUX process
séparés, avec redémarrage automatique (respawn) : un plantage de l'un
n'affecte jamais l'autre.

Author : Vincent FOUCAULT — Octobre 2026

Usage :
    ros2 launch listen_and_talk listen_and_talk.launch.py
    ros2 launch listen_and_talk listen_and_talk.launch.py listen:=false
    ros2 launch listen_and_talk listen_and_talk.launch.py params_file:=/chemin/autre.yaml
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('listen_and_talk')
    params = LaunchConfiguration('params_file')
    log_level = LaunchConfiguration('log_level')

    def node(name, enable_arg):
        return Node(
            package='listen_and_talk',
            executable=name,
            name=name,
            output='screen',
            emulate_tty=True,
            parameters=[params],
            arguments=['--ros-args', '--log-level', log_level],
            respawn=True,
            respawn_delay=2.0,
            condition=IfCondition(LaunchConfiguration(enable_arg)),
        )

    return LaunchDescription([
        DeclareLaunchArgument('params_file',
                              default_value=os.path.join(share, 'config', 'listen_and_talk.yaml')),
        DeclareLaunchArgument('listen', default_value='true'),
        DeclareLaunchArgument('talk', default_value='true'),
        DeclareLaunchArgument('log_level', default_value='info'),
        node('talk_node', 'talk'),
        node('listen_node', 'listen'),
    ])
