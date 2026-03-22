from setuptools import setup
import os
from glob import glob

package_name = 'qbo_games'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Launch files
        (os.path.join('share', package_name, 'launch'), 
         glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        # Config files
        (os.path.join('share', package_name, 'config'), 
         glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=[
        'setuptools',
        'opencv-python',
        'mediapipe',
        'numpy'
    ],
    zip_safe=True,
    maintainer='Vincent',
    maintainer_email='your.email@example.com',
    description='Package de jeu chifoumi pour le robot Neo avec détection de gestes',
    license='MIT',
    entry_points={
        'console_scripts': [
            'qbo_chifumi_node = qbo_games.qbo_chifumi:main',
            'chifumi_video_test = qbo_games.chifumi_video_test:main',
        ],  
    },
)