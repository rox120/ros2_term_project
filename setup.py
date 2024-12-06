from setuptools import setup
import os
from glob import glob

package_name = 'ros2_term_project'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/**')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ros2',
    maintainer_email='ros2@test.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'spawn_prius = ros2_term_project.spawn_prius:main',
            'starter = ros2_term_project.car_starter:main',
            'controller=ros2_term_project.car_controller:main',
            'line_follower=ros2_term_project.line_follower:main',
            'box_spawn=ros2_term_project.box_spawn:main',
            'teleop_manager=ros2_term_project.teleop_manager:main',
            'lidar_obstacle_avoidance = ros2_term_project.lidar_obstacle_avoidance:main',
        ],
    },
)
