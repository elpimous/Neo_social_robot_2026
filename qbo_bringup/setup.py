from setuptools import find_packages, setup

package_name = 'qbo_bringup'

setup(
    name=package_name,
    version='0.1.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/qbo_startup.launch.py',
        ]),
    ],
    install_requires=[
        "rclpy",
        "qbo_msgs",
        'setuptools',
    ],
    zip_safe=True,
    maintainer='zwolinski',
    maintainer_email='sylvain-zwolinski@orange.fr',
    description='The qbo_bringup package provides a unified launch system for bringing up the Qbo robot.',
    license='BSD-3-Clause',
    # tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'smach.py = qbo_bringup.smach:main',
        ],
    },
)
