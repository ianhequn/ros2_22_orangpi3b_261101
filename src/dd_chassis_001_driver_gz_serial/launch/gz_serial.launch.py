# 2026-11-23 u22
"""单独启动假 MCU 节点（通常由 gazebo.launch.py hwi_mode:=real 自动带上）。"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('dd_chassis_001_driver_gz_serial')
    default_yaml = os.path.join(pkg, 'config', 'gz_serial.yaml')

    params_file = LaunchConfiguration('params_file')
    serial_device = LaunchConfiguration('serial_device')

    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default_yaml,
                              description='gz_serial 参数文件'),
        DeclareLaunchArgument('serial_device', default_value='/dev/ttyV1',
                              description='串口设备（覆盖 yaml）'),
        Node(package='dd_chassis_001_driver_gz_serial',
             executable='gz_serial_node',
             name='dd_chassis_001_gz_serial',
             parameters=[params_file, {'serial_device': serial_device}],
             output='screen'),
    ])
