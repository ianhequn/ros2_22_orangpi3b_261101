#!/usr/bin/env python3
r"""
QoS 中继：把 /cmd_vel 的 Twist 转发到 diff_drive_controller 的输入话题.

背景（2026-11 实机踩坑）：
  diff_drive_controller 的输入话题硬编码成 ~/cmd_vel_unstamped，而 nav2、teleop
  都发 /cmd_vel，中间必须有人转发。缺这条中继时表现是"规划成功但车不动"：
  /cmd_vel 有发布者、cmd_vel_unstamped 零订阅者，两端都不报错。

QoS 说明（避免误判）：
  DDS 允许 RELIABLE 发布者投递给 BEST_EFFORT 订阅者，不兼容的是反向
  （BEST_EFFORT 发 -> RELIABLE 收，会打 incompatible QoS 警告并静默丢包）。
  本节点订阅端用 RELIABLE、发布端用 BEST_EFFORT，两个方向都兼容，并打印转发计数，
  比 topic hz 更能确认中继真的在工作（topic 信息可能来自缓存）。

示例：
  ros2 run dd_chassis_001_driver qos_relay.py \
      --in /cmd_vel --out /diff_drive_controller/cmd_vel_unstamped
"""

import argparse
import sys

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.utilities import remove_ros_args


class QosRelay(Node):
    def __init__(self, args):
        super().__init__('qos_relay')
        # 订阅端用 RELIABLE，兼容官方 teleop 的默认发布
        sub_qos = QoSProfile(depth=10)
        sub_qos.reliability = ReliabilityPolicy.RELIABLE
        sub_qos.history = HistoryPolicy.KEEP_LAST
        # 发布端用 BEST_EFFORT，匹配 diff_drive_controller 的订阅
        pub_qos = QoSProfile(depth=10)
        pub_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        pub_qos.history = HistoryPolicy.KEEP_LAST

        self.pub = self.create_publisher(Twist, args.out, pub_qos)
        self.create_subscription(Twist, args.in_, self.on_msg, sub_qos)
        self.count = 0
        self.get_logger().info(f'中继 {args.in_}(RELIABLE) -> {args.out}(BEST_EFFORT)')

    def on_msg(self, msg):
        self.pub.publish(msg)
        self.count += 1
        # 每秒提示一次，确认中继真的在转发（官方 teleop 不打印发布状态）
        if self.count % 100 == 0:
            self.get_logger().info(f'已转发 {self.count} 条')


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--in', dest='in_', default='/cmd_vel', help='输入话题')
    parser.add_argument('--out', default='/diff_drive_controller/cmd_vel_unstamped',
                        help='输出话题')
    args = parser.parse_args(remove_ros_args(sys.argv)[1:])

    rclpy.init()
    node = QosRelay(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
