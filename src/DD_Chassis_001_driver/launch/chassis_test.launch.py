#!/usr/bin/env python3
"""验证用启动：ros2_control_node + robot_state_publisher + 两个控制器。

硬件参数（串口/帧尾/标定）在 config/dd_chassis_001.ros2_control.xacro 里改。
用法：
    ros2 launch DD_Chassis_001_driver chassis_test.launch.py
    ros2 launch DD_Chassis_001_driver chassis_test.launch.py serial_debug:=false
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('DD_Chassis_001_driver')
    xacro_file = os.path.join(pkg_share, 'config', 'chassis_test.urdf.xacro')
    controllers_yaml = os.path.join(pkg_share, 'config', 'controllers.yaml')

    robot_description = ParameterValue(Command(['xacro ', xacro_file]), value_type=str)

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}],
    )

    # controller_manager：URDF 里的 <ros2_control> 块决定加载哪个硬件插件
    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        output='screen',
        parameters=[{'robot_description': robot_description}, controllers_yaml],
    )

    def spawner(name):
        return Node(
            package='controller_manager',
            executable='spawner',
            arguments=[name, '--controller-manager', '/controller_manager'],
            output='screen',
        )

    joint_state_broadcaster = spawner('joint_state_broadcaster')
    diff_drive_controller = spawner('diff_drive_controller')

    # 等 ros2_control_node 起来再加载控制器，避免 spawner 抢跑失败
    load_jsb = RegisterEventHandler(
        OnProcessStart(target_action=ros2_control_node, on_start=[joint_state_broadcaster]))
    load_ddc = RegisterEventHandler(
        OnProcessStart(target_action=joint_state_broadcaster, on_start=[diff_drive_controller]))

    return LaunchDescription([
        DeclareLaunchArgument(
            'serial_debug', default_value='true',
            description='是否启动 chassis_monitor 打表（订阅 /joint_states、/imu、odom）'),
        robot_state_publisher,
        ros2_control_node,
        load_jsb,
        load_ddc,
        Node(
            package='DD_Chassis_001_driver',
            executable='chassis_monitor.py',
            output='screen',
            condition=IfCondition(LaunchConfiguration('serial_debug')),
        ),
    ])
