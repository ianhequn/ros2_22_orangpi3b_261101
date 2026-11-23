import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command, LaunchConfiguration, PythonExpression)
from launch_ros.actions import Node


def generate_launch_description():
    """导航栈：EKF + 雷达过滤 + SLAM + Nav2 + RViz。

    只有一个 sim 开关，默认 true。它同时决定两件必须保持一致的事：

        sim=true   → use_sim_time=true   （用 Gazebo 的 /clock）
                     本文件不起 robot_state_publisher
                     （gazebo.launch.py 已经起了，再起会有两个节点抢 TF 树）

        sim=false  → use_sim_time=false  （真机没有 /clock）
                     本文件起 robot_state_publisher
                     （真机本机没有别的 RSP，TF 树必须由这里提供）

    拆成两个参数（use_sim_time + use_rsp）很容易配错：只传一个是
    最容易犯的错，而且后果是 EKF/SLAM/Nav2 一直等 /clock 不工作。

    用法：
        仿真   ros2 launch ... slam_nav2.launch.py
        真机   ros2 launch ... slam_nav2.launch.py sim:=false

    controller spawner 和 /cmd_vel relay 不在这里：
        仿真时它们在 gazebo.launch.py（底盘生命周期跟 Gazebo 绑定）；
        真机时 controller 在 orangepi，relay 由底盘驱动的 launch 负责。
    """
    pkg = get_package_share_directory('dd_chassis_01_description')
    nav2_pkg = get_package_share_directory('nav2_bringup')

    sim = LaunchConfiguration('sim')
    use_rviz = LaunchConfiguration('use_rviz')
    params_file = LaunchConfiguration('params_file')
    slam_params_file = LaunchConfiguration('slam_params_file')

    # sim 本身就是 true/false 字符串，可直接当 use_sim_time 用
    use_sim_time = sim

    # sim=false（真机）时才自己起 RSP
    use_rsp = PythonExpression(["str('", sim, "' != 'true').lower()"])

    # robot_description 只在 use_rsp=true 时才会被求值（Command 是惰性的，
    # 节点被 IfCondition 跳过时不会执行），仿真时不会有额外开销。
    #
    # hwi_mode 用 real：本文件不涉及 Gazebo，不需要 <gazebo> 段和
    # <ros2_control>。link/joint 树两种模式完全一样，TF 照常发布。
    xacro_file = os.path.join(pkg, 'urdf', 'dd_chassis_01.urdf.xacro')
    robot_desc = Command(['xacro ', xacro_file, ' hwi_mode:=real use_gazebo:=false'])

    rviz_config = os.path.join(
        nav2_pkg, 'rviz', 'nav2_default_view.rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'sim', default_value='true',
            description='true: 仿真（用 /clock，RSP 由 gazebo.launch.py 提供）| '
                        'false: 真机（无 /clock，本文件起 RSP）'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(pkg, 'config', 'nav2_params.yaml')),
        DeclareLaunchArgument(
            'slam_params_file',
            default_value=os.path.join(pkg, 'config', 'slam_lifelong.yaml')),

        # TF 树：base_footprint -> base_link -> ydlidar_link -> lidar_scan_link
        # 只在真机（sim=false）启动。仿真时由 gazebo.launch.py 提供。
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{
                'robot_description': robot_desc,
                'use_sim_time': use_sim_time,
            }],
            condition=IfCondition(use_rsp),
            output='screen',
        ),

        # 雷达过滤：屏蔽车体自身
        # 读 /scan_raw，写 /scan（slam 和 costmap 用）
        # 真机 ydlidar 驱动默认发 /scan，需要在那边 remap 成 /scan_raw
        Node(
            package='laser_filters',
            executable='scan_to_scan_filter_chain',
            name='scan_filter',
            output='screen',
            parameters=[
                os.path.join(pkg, 'config', 'scan_filter.yaml'),
                {'use_sim_time': use_sim_time},
            ],
            remappings=[('scan', 'scan_raw'), ('scan_filtered', 'scan')],
        ),

        # EKF：轮速 + IMU -> odom -> base_footprint
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[
                os.path.join(pkg, 'config', 'ekf.yaml'),
                {'use_sim_time': use_sim_time},
            ],
            remappings=[('odometry/filtered', 'odom')],
        ),

        # slam_toolbox 终身建图
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[
                slam_params_file,
                {'use_sim_time': use_sim_time},
            ],
        ),

        # Nav2（不含 map_server / amcl，地图由 slam_toolbox 提供）
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_pkg, 'launch', 'navigation_launch.py')),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'params_file': params_file,
                'autostart': 'true',
                'use_composition': 'False',
            }.items(),
        ),

        # RViz
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': use_sim_time}],
            condition=IfCondition(use_rviz),
            output='screen',
        ),
    ])
