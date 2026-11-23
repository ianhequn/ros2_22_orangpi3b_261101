# 2026-11-23 u22
"""假 MCU：把本地虚拟串口和 Gazebo 双向接起来。

链路：
    ros2_control 驱动  <->  /dev/ttyV0  ~  socat  ~  /dev/ttyV1  <->  本节点
    本节点  --std_msgs/Float64-->  ros_gz_bridge  -->  gz joint cmd_vel（轮子转）
    gz 传感器/关节反馈 -->  ros_gz_bridge --> 本节点组 64B 帧 --> 串口回去

它扮演的角色是真机上的 MCU 固件：
  * 解析 "sz qeiM <i> T <v>" 速度命令，转成 gz 关节速度指令；
  * 按 ~120Hz 上报 64B 定长帧（两个轮速 + 三个欧拉角 + 调试用的 PID 量）；
  * 帧尾魔数、字段顺序与真机完全一致，因此驱动侧代码一行都不用改。
"""
import math

import rclpy
import serial
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Float64

from .protocol import (DEFAULT_FRAME_END, CommandParser, encode_frame,
                       euler_from_quaternion)

MOTOR_COUNT = 2


class GzSerialNode(Node):

    def __init__(self):
        super().__init__('dd_chassis_001_gz_serial')

        # ---- 串口 ----
        self.declare_parameter('serial_device', '/dev/ttyV1')
        self.declare_parameter('baud_rate', 1000000)
        self.declare_parameter('serial_reopen_period_s', 1.0)
        # ---- 帧 ----
        self.declare_parameter('frame_end', DEFAULT_FRAME_END)
        self.declare_parameter('report_rate', 120.0)
        # ---- 电机号 -> gz 关节 ----
        self.declare_parameter('motor0_joint', 'wheel_r')
        self.declare_parameter('motor1_joint', 'wheel_l')
        self.declare_parameter('motor_speed_scale', [1.0, 1.0])
        self.declare_parameter('motor_invert', [False, False])
        # 假固件的 PID 参数：只用于填上报帧里的诊断字段，
        # 物理响应由 gz JointController 直接给理想速度（真机是固件 PID 调的）。
        self.declare_parameter('motor_kp', [1.0, 1.0])
        self.declare_parameter('motor_ki', [0.1, 0.1])
        self.declare_parameter('motor_kd', [0.0, 0.0])
        # ---- 话题 ----
        self.declare_parameter('joint_states_topic', 'mcu_proxy/joint_states')
        self.declare_parameter('imu_topic', 'mcu_proxy/imu')
        self.declare_parameter('debug_log', False)
        self.declare_parameter('motor_cmd_topics',
                               ['mcu_proxy/motor0_cmd_vel',
                                'mcu_proxy/motor1_cmd_vel'])

        self._device = self.get_parameter('serial_device').value
        self._baud = int(self.get_parameter('baud_rate').value)
        self._reopen_period = float(self.get_parameter('serial_reopen_period_s').value)
        self._frame_end = int(self.get_parameter('frame_end').value)
        self._report_rate = float(self.get_parameter('report_rate').value)
        self._joint_names = [self.get_parameter('motor0_joint').value,
                             self.get_parameter('motor1_joint').value]
        self._scales = list(self.get_parameter('motor_speed_scale').value)
        self._inverts = list(self.get_parameter('motor_invert').value)
        self._kp = list(self.get_parameter('motor_kp').value)
        self._ki = list(self.get_parameter('motor_ki').value)
        self._kd = list(self.get_parameter('motor_kd').value)
        cmd_topics = list(self.get_parameter('motor_cmd_topics').value)
        self._debug_log = bool(self.get_parameter('debug_log').value)

        self._parser = CommandParser()
        self._serial = None
        self._last_open_attempt = None   # None = 还没试过，首次立刻尝试
        self._last_tick_ns = None

        # 反馈缓存
        self._joint_velocity = [0.0, 0.0]      # 固件单位（= gz rad/s / scale）
        self._joint_position = [0.0, 0.0]
        self._target = [0.0, 0.0]              # 刚才下发的速度指令（固件单位）
        self._integral = [0.0, 0.0]
        self._prev_error = [0.0, 0.0]
        self._output = [0.0, 0.0]
        self._imu_euler_deg = [0.0, 0.0, 0.0]
        self._have_joint_state = False
        self._have_imu = False

        self._cmd_pubs = [
            self.create_publisher(Float64, cmd_topics[i], 10) for i in range(MOTOR_COUNT)
        ]
        self.create_subscription(
            JointState, self.get_parameter('joint_states_topic').value,
            self._on_joint_state, 10)
        self.create_subscription(
            Imu, self.get_parameter('imu_topic').value, self._on_imu, 10)

        self.create_timer(1.0 / self._report_rate, self._on_timer)
        # 串口打不开时也要持续重试，因此打开动作放在定时器里
        self._try_open()
        self.get_logger().info(
            f'假 MCU 已启动: {self._device}@{self._baud}, 上报 {self._report_rate:.0f}Hz, '
            f'motor0->{self._joint_names[0]}, motor1->{self._joint_names[1]}')

    # ------------------------------------------------------------------ 串口
    def _try_open(self):
        now = self.get_clock().now()
        if (self._last_open_attempt is not None
                and (now - self._last_open_attempt).nanoseconds < self._reopen_period * 1e9):
            return
        self._last_open_attempt = now
        try:
            # write_timeout 必须显式设为有限值：pyserial 默认阻塞写，
            # 对端（驱动/内核缓冲）一时不收就会把整个定时器卡住几百毫秒，
            # 驱动那边就会判定"通信中断"。宁可丢一帧也不能卡。
            self._serial = serial.Serial(
                self._device, self._baud, timeout=0, write_timeout=0.05)
            self._serial.reset_input_buffer()
            self._parser.reset()
            self.get_logger().info(f'串口已打开: {self._device}')
        except (serial.SerialException, OSError) as exc:
            self._serial = None
            self.get_logger().error(
                f'打开 {self._device} 失败: {exc}（{self._reopen_period:.1f}s 后重试）',
                throttle_duration_sec=5.0)

    def _read_commands(self):
        if self._serial is None:
            return
        try:
            # 一周期内最多读这么多次，防止对端狂刷导致本回调超时
            for _ in range(8):
                chunk = self._serial.read(256)
                if not chunk:
                    break
                for motor, value in self._parser.feed(chunk):
                    if not 0 <= motor < MOTOR_COUNT:
                        self.get_logger().warn(f'忽略未知电机号 {motor}',
                                               throttle_duration_sec=5.0)
                        continue
                    self._apply_command(motor, value)
        except (serial.SerialException, OSError) as exc:
            self.get_logger().error(f'串口读失败: {exc}，关闭并重连')
            self._close_serial()

    def _apply_command(self, motor, value):
        if not math.isfinite(value):
            return
        self._target[motor] = value
        # 固件速度值 -> gz 关节速度：joint = speed * scale * invert
        joint_velocity = value * self._scales[motor] * (-1.0 if self._inverts[motor] else 1.0)
        msg = Float64()
        msg.data = joint_velocity
        self._cmd_pubs[motor].publish(msg)
        if self._debug_log:
            self.get_logger().info(
                f'收到命令 motor{motor} speed={value:.4f} -> {self._joint_names[motor]} '
                f'{joint_velocity:.4f} rad/s')

    def _close_serial(self):
        if self._serial is not None:
            try:
                self._serial.close()
            except (serial.SerialException, OSError):
                pass
        self._serial = None
        self._last_open_attempt = None   # None = 还没试过，首次立刻尝试
        self._last_tick_ns = None

    # ------------------------------------------------------------------ 反馈
    def _on_joint_state(self, msg: JointState):
        for motor, joint in enumerate(self._joint_names):
            if joint in msg.name:
                index = msg.name.index(joint)
                if index < len(msg.velocity) and math.isfinite(msg.velocity[index]):
                    # gz rad/s -> 固件单位
                    self._joint_velocity[motor] = (
                        msg.velocity[index]
                        * (-1.0 if self._inverts[motor] else 1.0) / self._scales[motor])
                if index < len(msg.position) and math.isfinite(msg.position[index]):
                    self._joint_position[motor] = msg.position[index]
        self._have_joint_state = True

    def _on_imu(self, msg: Imu):
        q = msg.orientation
        roll, pitch, yaw = euler_from_quaternion(q.x, q.y, q.z, q.w)
        self._imu_euler_deg = [math.degrees(roll), math.degrees(pitch), math.degrees(yaw)]
        self._have_imu = True

    # ------------------------------------------------------------------ 组帧
    def _update_pid_diagnostics(self):
        dt = 1.0 / self._report_rate
        for motor in range(MOTOR_COUNT):
            error = self._target[motor] - self._joint_velocity[motor]
            self._integral[motor] = max(-100.0, min(100.0,
                                                    self._integral[motor] + error * dt))
            derivative = (error - self._prev_error[motor]) / dt
            self._output[motor] = (self._kp[motor] * error
                                   + self._ki[motor] * self._integral[motor]
                                   + self._kd[motor] * derivative)
            self._prev_error[motor] = error

    def _on_timer(self):
        now_ns = self.get_clock().now().nanoseconds
        if self._last_tick_ns is not None:
            gap_ms = (now_ns - self._last_tick_ns) / 1e6
            if gap_ms > 30.0:
                self.get_logger().warn(
                    f'上报节拍被推迟 {gap_ms:.0f}ms（会让驱动误判通信中断）',
                    throttle_duration_sec=1.0)
        self._last_tick_ns = now_ns

        if self._serial is None:
            self._try_open()
            if self._serial is None:
                return

        self._read_commands()
        if self._serial is None:
            return

        self._update_pid_diagnostics()
        frame = encode_frame(
            imu=self._imu_euler_deg,
            error=[self._target[i] - self._joint_velocity[i] for i in range(MOTOR_COUNT)],
            output=self._output,
            kp=self._kp,
            ki=self._ki,
            kd=self._kd,
            speed=self._joint_velocity,
            frame_end=self._frame_end,
        )
        try:
            self._serial.write(frame)
        except serial.SerialTimeoutException:
            # 对端背压：丢这一帧，下个周期再发，绝不阻塞
            self.get_logger().warn(
                '串口写超时，丢弃该帧', throttle_duration_sec=2.0)
            return
        except (serial.SerialException, OSError) as exc:
            self.get_logger().error(f'串口写失败: {exc}，关闭并重连')
            self._close_serial()

    def destroy_node(self):
        self._close_serial()
        super().destroy_node()


def main(argv=None):
    rclpy.init(args=argv)
    node = GzSerialNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
