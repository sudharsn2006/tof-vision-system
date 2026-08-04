from setuptools import setup
import os
from glob import glob

package_name = 'tof_lidar_bringup'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name] if os.path.exists('resource/' + package_name) else []),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Sudharsan',
    maintainer_email='sudharsan@example.com',
    description='ROS 2 Sensor Fusion package for ToF Camera & YDLIDAR X2',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'tof_camera_node = tof_lidar_bringup.tof_camera_node:main',
            'ydlidar_node = tof_lidar_bringup.ydlidar_node:main',
            'sensor_fusion_node = tof_lidar_bringup.sensor_fusion_node:main',
            'rviz_visualizer = tof_lidar_bringup.rviz_visualizer:main',
        ],
    },
)
