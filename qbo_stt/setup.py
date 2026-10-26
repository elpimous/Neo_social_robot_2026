#!/usr/bin/env python3
"""
╔══════════════════════════════════════════════════════════════════╗
║   qbo_stt — setup.py                                             ║
║   Package ROS2 Speech-To-Text pour robot Néo                     ║
║   Author : Vincent Foucault — Avril 2026 — MIT                   ║
╚══════════════════════════════════════════════════════════════════╝
"""

import os
from glob import glob
from setuptools import setup, find_packages

package_name = 'qbo_stt'

setup(
    name=package_name,
    version='0.19.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # Index ROS2 standard
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # ── Launch files ─────────────────────────────────────────────────────
        # Tous les .launch.py dans launch/ → share/qbo_stt/launch/
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*.launch.py'))),

        # ── Config files (params.yaml, etc.) ─────────────────────────────────
        # Tous les .yaml dans config/ → share/qbo_stt/config/
        (os.path.join('share', package_name, 'config'),
            glob(os.path.join('config', '*.yaml'))),

        # ── Modèles ONNX (optionnel) ─────────────────────────────────────────
        # Décommente si tu veux que les modèles soient installés via colcon.
        # En pratique, les modèles ONNX pèsent 200+ Mo, il est plus pratique
        # de les laisser hors du package (chemin absolu dans params.yaml).
        # (os.path.join('share', package_name, 'models'),
        #     glob(os.path.join('models', '*.onnx'))),
        # (os.path.join('share', package_name, 'models'),
        #     glob(os.path.join('models', '*.txt'))),
    ],
    install_requires=[
        'setuptools',
        'numpy',
        'onnxruntime-gpu',
        'pyaudio',
        'librosa',
    ],
    zip_safe=True,
    maintainer='Vincent Foucault',
    maintainer_email='vincent.foucault@example.com',
    description='Speech-To-Text node — FastConformer CTC CUDA + Silero VAD, robot Néo',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'stt_node = qbo_stt.stt_node:main',
        ],
    },
)