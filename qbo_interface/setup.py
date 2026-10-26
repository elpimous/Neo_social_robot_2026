from setuptools import find_packages, setup

package_name = 'qbo_interface'

setup(
    name=package_name,
    version='0.1.1',
    packages=find_packages(exclude=['test'], include=['qbo_interface', 'qbo_interface.*']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/qbo_interface/launch', ['launch/smach_qbo.launch.py']),
        ('share/qbo_interface/config', []),
        ('share/qbo_interface/web', [
            'web/index.html',
            'web/style.css',
            'web/script.js'
        ]),
    ],
    install_requires=[
        "rclpy",
        "numpy",
        "qbo_msgs",
    ],
    zip_safe=True,
    maintainer='zwolinski',
    maintainer_email='sylvain-zwolinski@orange.fr',
    description='Supervisory SMACH node and web interface for Qbo robot.',
    license='BSD-3-Clause',
    # tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'smach_qbo = qbo_interface.smach_qbo:main',
        ],
    },
)
