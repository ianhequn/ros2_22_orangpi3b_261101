#!/usr/bin/env python3
"""
orangepi 侧硬件接口启动（只跑硬件接口，不发布 TF）.

分工：
    orangepi（本 launch）：
        ros2_control_node  -> 加载 DDChassis001System 硬件插件
        joint_state_broadcaster
        diff_drive_controller
    导航侧（WSL2 的 robot.launch.py）：
        robot_state_publisher（TF 树）、scan_filter、EKF、slam_toolbox、Nav2、RViz

所以这里**不起 robot_state_publisher**。两边各发一份会让
base_footprint->base_link 出现两个不同值（测试 xacro 0.0325 vs
真实 URDF 0.0665），导致雷达高度算错、scan_filter 的 box 判定偏移。

robot_description 仍然要传给 ros2_control_node，硬件插件需要它
来解析 wheel_l / wheel_r 的关节定义。

用法：
    ros2 launch dd_chassis_001_driver chassis_hw.launch.py
    ros2 launch dd_chassis_001_driver chassis_hw.launch.py monitor:=false
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessStart
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('dd_chassis_001_driver')
    # 用本包自己的 xacro：串口设备、speed_scale 等标定值都在
    # config/dd_chassis_001.ros2_control.xacro 里，不要另外维护一份 URDF。
    # chassis_test.urdf.xacro 是极简测试模型，只用它的关节结构；
    # 硬件参数全部来自 include 进来的 ros2_control 片段。
    xacro_file = os.path.join(pkg_share, 'config', 'chassis_test.urdf.xacro')
    controllers_yaml = os.path.join(pkg_share, 'config', 'controllers.yaml')

    robot_description = ParameterValue(Command(['xacro ', xacro_file]), value_type=str)

    # controller_manager：URDF 里的 <ros2_control> 决定加载哪个硬件插件
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

    # 按顺序加载控制器，避免 spawner 抢跑
    load_jsb = RegisterEventHandler(
        OnProcessStart(target_action=ros2_control_node, on_start=[joint_state_broadcaster]))
    load_ddc = RegisterEventHandler(
        OnProcessStart(target_action=joint_state_broadcaster, on_start=[diff_drive_controller]))

    return LaunchDescription([
        DeclareLaunchArgument(
            'monitor', default_value='false',
            description='是否启动 chassis_monitor 打表（订阅 /joint_states、/imu、odom）'),
        ros2_control_node,
        load_jsb,
        load_ddc,
        Node(
            package='dd_chassis_001_driver',
            executable='chassis_monitor.py',
            output='screen',
            condition=IfCondition(LaunchConfiguration('monitor')),
        ),
    ])
