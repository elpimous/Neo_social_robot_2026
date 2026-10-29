# Auteur : Vincent FOUCAULT
import os
from glob import glob
from setuptools import setup

package_name = 'qbo_nav_waypoints'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md', 'COMMANDES.md']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
        (os.path.join('share', package_name, 'scripts'), glob('scripts/*.sh')),
        (os.path.join('share', package_name, 'behavior_trees'), glob('behavior_trees/*.xml')),
        (os.path.join('share', package_name, 'maps'),
         glob('maps/*.yaml') + glob('maps/*.pgm') + glob('maps/*.png') + glob('maps/*.md')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Vincent FOUCAULT',
    maintainer_email='elpimous12@gmail.com',
    description="Néo (QBo) — test d'endurance navigation laser sur waypoints",
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'waypoint_mission = qbo_nav_waypoints.waypoint_mission:main',
            'record_waypoints = qbo_nav_waypoints.record_waypoints:main',
            'head_laser_tf = qbo_nav_waypoints.head_laser_tf:main',
            'head_lock = qbo_nav_waypoints.head_lock:main',
            'calibrate_head_level = qbo_nav_waypoints.calibrate_head_level:main',
            'diagnose_base_lidar = qbo_nav_waypoints.diagnose_base_lidar:main',
            'neo_monitor = qbo_nav_waypoints.neo_monitor:main',
            'scan_range_filter = qbo_nav_waypoints.scan_range_filter:main',
            'speed_trace = qbo_nav_waypoints.speed_trace:main',
        ],
    },
)
