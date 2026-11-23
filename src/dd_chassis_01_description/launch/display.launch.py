import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """纯 RViz 看模型：不起 Gazebo，用 joint_state_publisher_gui 拖滑块。

    用 hwi_mode='real' + use_gazebo='false' 展开：几何和关节树跟仿真完全一样，
    但不生成 <gazebo> 段。RViz 本来就会忽略这些标签，去掉只是让喂给
    robot_state_publisher 的 URDF 更干净、更容易排查。
    """
    pkg = get_package_share_directory('dd_chassis_01_description')
    xacro_file = os.path.join(pkg, 'urdf', 'dd_chassis_01.urdf.xacro')

    doc = xacro.process_file(
        xacro_file,
        mappings={'hwi_mode': 'real', 'use_gazebo': 'false', 'prefix': ''})
    robot_desc = doc.toprettyxml(indent='  ')

    return LaunchDescription([
        Node(package='robot_state_publisher',
             executable='robot_state_publisher',
             parameters=[{'robot_description': robot_desc}],
             output='screen'),

        Node(package='joint_state_publisher_gui',
             executable='joint_state_publisher_gui'),

        Node(package='rviz2',
             executable='rviz2',
             arguments=['-d',
                        os.path.join(pkg,
                        'launch', 'display.rviz')]),
    ])
