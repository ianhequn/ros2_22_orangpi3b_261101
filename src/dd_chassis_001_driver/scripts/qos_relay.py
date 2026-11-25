#!/usr/bin/env python3
"""
QoS 中继：把 RELIABLE 发布的 Twist 转成 BEST_EFFORT 转发。

为什么需要：diff_drive_controller 的 cmd_vel 订阅固定 BEST_EFFORT，
而 teleop_twist_keyboard 2.4.1（humble 官方包）没有 qos 参数，固定用默认
RELIABLE 发布 —— DDS 规定 RELIABLE 发布者不能向 BEST_EFFORT 订阅者投递，
消息被静默丢弃、且两端都不报错。用本节点收下 RELIABLE、改用 BEST_EFFORT 转发。

示例：
  ros2 run dd_chassis_001_driver qos_relay.py \\
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
