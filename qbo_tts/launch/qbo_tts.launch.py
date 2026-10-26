#!/usr/bin/env python3
"""
qbo_tts — qbo_tts.launch.py
Lance le node TTS de Néo avec config/qbo_tts_params.yaml et les modèles installés.
Author : Vincent FOUCAULT — Octobre 2026

Usage :
    ros2 launch qbo_tts qbo_tts.launch.py
    ros2 launch qbo_tts qbo_tts.launch.py params_file:=/chemin/autre.yaml
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('qbo_tts')
    models = os.path.join(share, 'models')

    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(share, 'config', 'qbo_tts_params.yaml'),
        description='Fichier de paramètres du TTS',
    )

    tts_node = Node(
        package='qbo_tts',
        executable='qbo_tts_node',
        name='qbo_tts_node',
        output='screen',
        emulate_tty=True,
        parameters=[
            LaunchConfiguration('params_file'),
            {
                'model_path':      os.path.join(models, 'axel.onnx'),
                'tokens_path':     os.path.join(models, 'tokens.txt'),
                'espeak_data_dir': os.path.join(models, 'espeak-ng-data'),
            },
        ],
    )

    return LaunchDescription([params_file_arg, tts_node])
