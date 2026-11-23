import os

from ament_index_python.packages import (
    get_package_prefix, get_package_share_directory)
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import (
    Command, LaunchConfiguration, PythonExpression)
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('dd_chassis_01_description')
    xacro_file = os.path.join(pkg, 'urdf', 'dd_chassis_01.urdf.xacro')
    world = os.path.join(pkg, 'world', 'dd_chassis_01.world')
    controllers_yaml = os.path.join(pkg, 'config', 'controllers.yaml')
    bridge_sim_yaml = os.path.join(pkg, 'config', 'bridge.yaml')
    bridge_real_yaml = os.path.join(pkg, 'config', 'bridge_real.yaml')

    gz_serial_pkg = get_package_share_directory('dd_chassis_001_driver_gz_serial')
    gz_serial_yaml = os.path.join(gz_serial_pkg, 'config', 'gz_serial.yaml')

    # ------------------------------------------------------------------
    # hwi_mode：选择 ros2_control 的硬件插件
    #   sim  -> ign_ros2_control/IgnitionSystem（Gazebo 内建，纯仿真）
    #   real -> dd_chassis_001_driver/DDChassis001System（本机开发用：
    #           硬件插件走虚拟串口，串口转发节点再与 Gazebo 话题通信，
    #           以此模拟 orangepi 上的 MCU 固件）
    #
    # real 模式的完整链路（与真机只差"串口另一端是谁"）：
    #   Gazebo 物理  <-  JointController  <-  gz_serial  <-  /dev/ttyV1
    #        ||                                                  ~ socat ~
    #   Gazebo 传感器 -> gz_serial -> 64B 帧 -> /dev/ttyV0 <-> dd_chassis_001_driver
    # 串口对由 systemd 的 my_virtual_serial.service（socat）提供。
    #
    # 真机部署时不要用本 launch，只在 orangepi 起 ros2_control，
    # 本机改用 slam_nav2.launch.py。
    # ------------------------------------------------------------------
    hwi_mode = LaunchConfiguration('hwi_mode')
    prefix = LaunchConfiguration('prefix')
    serial_device = LaunchConfiguration('serial_device')
    proxy_serial_device = LaunchConfiguration('proxy_serial_device')

    is_real = PythonExpression(["'", hwi_mode, "' == 'real'"])
    is_sim = PythonExpression(["'", hwi_mode, "' == 'sim'"])

    # 用 Command 而不是 xacro.process_file()：
    #   process_file 在 generate_launch_description() 里执行，那时
    #   hwi_mode 这个 launch 参数还没解析，拿不到值。
    #   Command 是运行时替换，能正确拿到 LaunchConfiguration。
    # 注意 controllers_yaml 必须传绝对路径：xacro 展开后
    # ign_ros2_control 插件不会再解析 $(find ...)。
    robot_desc = Command([
        'xacro ', xacro_file,
        ' hwi_mode:=', hwi_mode,
        ' prefix:=', prefix,
        ' serial_device:=', serial_device,
        ' controllers_yaml:=', controllers_yaml,
    ])

    env = dict(os.environ)
    env['IGN_GAZEBO_RESOURCE_PATH'] = (
        os.path.dirname(pkg) + ':'
        + env.get('IGN_GAZEBO_RESOURCE_PATH', ''))

    # ign_ros2_control 的 .so 在 /opt/ros/humble/lib 下，但 ign-gazebo 的
    # SystemLoader 不搜索 LD_LIBRARY_PATH，只搜索
    # IGN_GAZEBO_SYSTEM_PLUGIN_PATH。不设这个变量就会报：
    #   [Err] [SystemLoader.cc:94] Failed to load system plugin
    #         [ign_ros2_control-system] : couldn't find shared library.
    # 后果是 controller_manager 根本不会被创建，所有 controller
    # spawner 永远空等，表现为"遥控完全没反应"。
    env['IGN_GAZEBO_SYSTEM_PLUGIN_PATH'] = (
        os.path.join(get_package_prefix('ign_ros2_control'), 'lib') + ':'
        + env.get('IGN_GAZEBO_SYSTEM_PLUGIN_PATH', ''))
    # WSLg 下 ogre2 渲染所需的三个变量（实测有效）
    env['MESA_D3D12_DEFAULT_ADAPTER_NAME'] = 'NVIDIA'   # 选 RTX 4060 独显
    env['MESA_GL_VERSION_OVERRIDE'] = '4.5'             # 绕过 Ogre2 copyTo 分支缺陷
    env['MESA_GLSL_VERSION_OVERRIDE'] = '450'

    return LaunchDescription([
        DeclareLaunchArgument(
            'hwi_mode', default_value='sim',
            description='sim: ign_ros2_control 纯仿真 | '
                        'real: 本机硬件插件 + 串口转发（模拟 MCU）'),
        DeclareLaunchArgument(
            'prefix', default_value='',
            description='link/joint 名前缀，多机器人时使用'),
        DeclareLaunchArgument(
            'serial_device', default_value='/dev/ttyV0',
            description='hwi_mode=real 时硬件插件打开的串口（虚拟串口一端）'),
        DeclareLaunchArgument(
            'proxy_serial_device', default_value='/dev/ttyV1',
            description='hwi_mode=real 时假 MCU 节点打开的串口（虚拟串口另一端）'),

        # Gazebo Fortress
        ExecuteProcess(
            cmd=['ign', 'gazebo', '-r', '-v', '4', '-s',
                 world],
            additional_env=env,
            output='screen'),

        ExecuteProcess(
            cmd=['ign', 'gazebo', '-v', '4', '-g'],
            additional_env=env,
            output='screen'),

        # 把展开后的 URDF 塞进 Gazebo。
        # 用 -string 而不是 -file：文件是 .xacro，ros_gz_sim 不会展开它。
        ExecuteProcess(
            cmd=['ros2', 'run',
                 'ros_gz_sim', 'create',
                 '-world', 'default',
                 '-string', robot_desc,
                 '-name', 'dd_chassis',
                 '-z', '0.05'],
            additional_env=env,
            output='screen'),

        # 话题桥。real 模式必须换一份配置：Gazebo 的 IMU 要改名绕到假 MCU，
        # 否则会和驱动自己发的 /imu 撞名。
        Node(package='ros_gz_bridge',
             executable='parameter_bridge',
             parameters=[{'config_file': bridge_sim_yaml}],
             condition=IfCondition(is_sim),
             additional_env=env,
             output='screen'),

        Node(package='ros_gz_bridge',
             executable='parameter_bridge',
             parameters=[{'config_file': bridge_real_yaml}],
             condition=IfCondition(is_real),
             additional_env=env,
             output='screen'),

        # ==================================================================
        # real 模式专属
        # ==================================================================

        # 假 MCU：虚拟串口 <-> Gazebo。它扮演固件，驱动完全不知情。
        Node(package='dd_chassis_001_driver_gz_serial',
             executable='gz_serial_node',
             name='dd_chassis_001_gz_serial',
             parameters=[gz_serial_yaml,
                         {'serial_device': proxy_serial_device}],
             condition=IfCondition(is_real),
             output='screen'),

        # real 模式的 controller_manager。sim 模式下它由 Gazebo 里的
        # ign_ros2_control 插件提供，这里不能再起一个（会抢 controller）。
        Node(package='controller_manager',
             executable='ros2_control_node',
             parameters=[controllers_yaml,
                         {'robot_description': robot_desc,
                          'use_sim_time': True}],
             condition=IfCondition(is_real),
             output='screen'),

        # ==================================================================
        # 共用
        # ==================================================================

        # 发布 robot_description（给 RViz / Nav2 用）
        # use_sim_time 必须为 true，否则 TF 时间戳与 /clock 不同步
        Node(package='robot_state_publisher',
             executable='robot_state_publisher',
             parameters=[{'robot_description': robot_desc,
                          'use_sim_time': True}],
             output='screen'),

        # ==================================================================
        # controller spawner
        #
        # 这两个原来放在 slam_nav2.launch.py 里，导致只启动本 launch 时
        # diff_drive_controller 根本没被 spawn，/cmd_vel_unstamped 无人订阅，
        # 表现为"遥控完全没反应"。放在这里才能保证 Gazebo 单独跑时底盘可控。
        #
        # spawner 内部会等 controller_manager 的 service 就绪，所以不需要
        # 额外加延时。slam_nav2.launch.py 里已移除这两个节点，避免重复 spawn。
        # 两种 hwi_mode 都适用：sim 的 controller_manager 在 Gazebo 里，
        # real 的在上面那个 ros2_control_node 里。
        # ==================================================================
        Node(package='controller_manager', executable='spawner',
             arguments=['joint_state_broadcaster'],
             output='screen'),

        Node(package='controller_manager', executable='spawner',
             arguments=['diff_drive_controller'],
             output='screen'),

        # /cmd_vel -> /diff_drive_controller/cmd_vel_unstamped
        #
        # 跟 slam_nav2 里那条 relay 同一个作用。放这里是为了 Gazebo 单独
        # 启动时遥控也能直接用（teleop 默认发 /cmd_vel）。
        Node(package='topic_tools', executable='relay',
             name='cmd_vel_relay',
             arguments=['/cmd_vel',
                        '/diff_drive_controller/cmd_vel_unstamped'],
             output='screen'),
    ])
