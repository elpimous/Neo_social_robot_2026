from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'qbo_discussion'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nvidia',
    maintainer_email='elpimous12@gmail.com',
    description='Package ROS2 pour la discussion avec QBO - personnalité:Néo - via RAG',
    license='Apache License 2.0',
    entry_points={
    'console_scripts': [
        'qbo_discussion_node = qbo_discussion.qbo_discussion:main',
        ],
    },
)
