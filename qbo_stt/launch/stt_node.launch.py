#!/usr/bin/env python3
"""
╔══════════════════════════════════════════════════════════════════╗
║   qbo_stt — stt_node.launch.py                                   ║
║   Lance le node STT FastConformer CTC + Silero VAD               ║
║   en chargeant automatiquement config/params.yaml                ║
║   Author : Vincent Foucault — Avril 2026 — MIT                   ║
╚══════════════════════════════════════════════════════════════════╝

Usage :
    # Lancement standard (charge params.yaml)
    ros2 launch qbo_stt stt_node.launch.py

    # Activer le debug VAD ponctuellement
    ros2 launch qbo_stt stt_node.launch.py vad_debug:=true

    # Changer le niveau de log
    ros2 launch qbo_stt stt_node.launch.py log_level:=debug

    # Override d'un paramètre ponctuel (via CLI après --)
    ros2 launch qbo_stt stt_node.launch.py \
        vad_debug:=true \
        log_level:=info
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # ── Arguments CLI ─────────────────────────────────────────────────────────
    # Ils peuvent être passés depuis la commande ros2 launch, par ex :
    #   ros2 launch qbo_stt stt_node.launch.py vad_debug:=true
    vad_debug_arg = DeclareLaunchArgument(
        'vad_debug',
        default_value='false',
        description='Active les logs debug détaillés en état MAYBE_END (true/false)',
    )

    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Niveau de log ROS2 : debug, info, warn, error, fatal',
    )

    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('qbo_stt'),
            'config',
            'params.yaml',
        ]),
        description='Chemin vers le fichier de paramètres YAML',
    )

    # ── Node stt_node ─────────────────────────────────────────────────────────
    # Les paramètres sont chargés depuis params.yaml, puis les valeurs
    # définies par les arguments CLI viennent les OVERRIDE ponctuellement
    # (utile pour vad_debug par exemple sans avoir à éditer le YAML).
    stt_node = Node(
        package='qbo_stt',
        executable='stt_node',
        name='stt_node',
        output='screen',
        emulate_tty=True,
        parameters=[
            LaunchConfiguration('params_file'),
            {
                'vad_debug': LaunchConfiguration('vad_debug'),
            },
        ],
        arguments=[
            '--ros-args',
            '--log-level', LaunchConfiguration('log_level'),
        ],
    )

    return LaunchDescription([
        vad_debug_arg,
        log_level_arg,
        params_file_arg,
        stt_node,
    ])
