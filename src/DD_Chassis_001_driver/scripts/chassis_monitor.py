#!/usr/bin/env python3
"""底盘数据打表：订阅 /joint_states、/imu、/diff_drive_controller/odom，单行实时刷新。

用于验证第 2 级（数据流出）与第 3 级（指令闭环）：手转轮子、转动车身、发 cmd_vel 都能从这里看出来。

示例：
  ros2 run DD_Chassis_001_driver chassis_monitor
  ros2 run DD_Chassis_001_driver chassis_monitor --imu-topic /imu --odom-topic /diff_drive_controller/odom
"""

import argparse
import math
import sys

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import Imu, JointState


class ChassisMonitor(Node):
    def __init__(self, args):
        super().__init__('chassis_monitor')
        self.left_name = args.left_joint
        self.right_name = args.right_joint
        self.vel = {args.left_joint: 0.0, args.right_joint: 0.0}
        self.yaw_deg = 0.0
        self.yaw_rate = 0.0
        self.odom_vx = 0.0
        self.odom_wz = 0.0
        self.last_imu_time = None
        self.cmd_vx_hold = args.cmd_vx
        self.cmd_wz_hold = args.cmd_wz
        self.cmd_pub = None
        if args.cmd_vx or args.cmd_wz:
            self.cmd_pub = self.create_publisher(Twist, args.cmd_topic, 10)

        self.create_subscription(JointState, '/joint_states', self.on_joints, 10)
        self.create_subscription(Imu, args.imu_topic, self.on_imu, 10)
        self.create_subscription(Odometry, args.odom_topic, self.on_odom, 10)
        self.create_timer(0.2, self.print_row)
        self.get_logger().info('等待 /joint_states、/imu、odom ...')

    def on_joints(self, msg):
        for name, position in zip(msg.name, msg.velocity):
            if name in self.vel:
                self.vel[name] = position

    def on_imu(self, msg):
        q = msg.orientation
        # 四元数只有 yaw 分量时可用此简化式，避免引入 tf 依赖
        self.yaw_deg = math.degrees(2.0 * math.atan2(q.z, q.w))
        self.yaw_rate = msg.angular_velocity.z
        self.last_imu_time = msg.header.stamp

    def on_odom(self, msg):
        self.odom_vx = msg.twist.twist.linear.x
        self.odom_wz = msg.twist.twist.angular.z

    def print_row(self):
        if self.cmd_pub is not None:
            cmd = Twist()
            cmd.linear.x = self.cmd_vx_hold
            cmd.angular.z = self.cmd_wz_hold
            self.cmd_pub.publish(cmd)

        print(
            f'\r关节速度[rad/s] {self.left_name}={self.vel[self.left_name]:+7.3f} '
            f'{self.right_name}={self.vel[self.right_name]:+7.3f} | '
            f'yaw={self.yaw_deg:+7.2f}° yaw_rate={self.yaw_rate:+6.3f} | '
            f'odom vx={self.odom_vx:+6.3f} wz={self.odom_wz:+6.3f}   ',
            end='', flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--left-joint', default='wheel_l')
    parser.add_argument('--right-joint', default='wheel_r')
    parser.add_argument('--imu-topic', default='/imu')
    parser.add_argument('--odom-topic', default='/diff_drive_controller/odom')
    parser.add_argument('--cmd-topic', default='/diff_drive_controller/cmd_vel_unstamped')
    parser.add_argument('--cmd-vx', type=float, default=0.0, help='持续发布线速度(m/s)，0 表示不发布')
    parser.add_argument('--cmd-wz', type=float, default=0.0, help='持续发布角速度(rad/s)')
    # launch/ros2 run 会自动追加 --ros-args ...，argparse 不认，先剔掉
    args = parser.parse_args(remove_ros_args(sys.argv)[1:])

    rclpy.init()
    node = ChassisMonitor(args)
    if node.cmd_pub is not None:
        node.get_logger().info(f'持续发布 {args.cmd_topic}: vx={args.cmd_vx} wz={args.cmd_wz}')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
