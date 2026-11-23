# 2026-11-23 u22
"""DD_Chassis_001 MCU 串口协议（Python 侧，与 C++ 驱动保持一致）。

帧：16 x 32bit = 64B，无帧头，末 4B 是魔数（默认 0x7F800000，按 IEEE754 即 +inf）。
命令：ASCII 行，'\\n' 结尾，形如 "sz qeiM 0 T 10.0"。
"""
import re
import struct

FRAME_FIELD_COUNT = 16
FRAME_SIZE = 64
DEFAULT_FRAME_END = 0x7F800000

# 15 个 float + 1 个 uint32，小端
_FRAME_STRUCT = struct.Struct('<15fI')

# 固件只认大写 SZ（小写会被静默丢弃，见 note/mcu.md）。
# 这里同时接受小写，是为了兼容早期文档/手敲命令，便于对比排查。
_CMD_RE = re.compile(
    r'^SZ\s+qeiM\s+(\d+)\s+T\s+(-?(?:\d+\.?\d*|\.\d+)(?:[eE][-+]?\d+)?)\s*$',
    re.IGNORECASE)


class CommandParser:
    """粘包/半包安全的行解析器。"""

    def __init__(self):
        self._buffer = ''

    def reset(self):
        self._buffer = ''

    def feed(self, data: bytes):
        """喂入原始字节，返回 [(motor_index, value), ...]。"""
        if not data:
            return []
        # 固件只发 ASCII；置 errors='ignore' 防止脏字节抛异常
        self._buffer += data.decode('ascii', errors='ignore')
        commands = []
        while '\n' in self._buffer:
            line, self._buffer = self._buffer.split('\n', 1)
            match = _CMD_RE.match(line.strip().replace('\r', ''))
            if match:
                commands.append((int(match.group(1)), float(match.group(2))))
        # 防御：对端一直不发换行时不要让缓冲无限增长
        if len(self._buffer) > 4096:
            self._buffer = ''
        return commands


def encode_frame(imu, error, output, kp, ki, kd, speed, frame_end=DEFAULT_FRAME_END):
    """组一帧 64B 上报数据。

    imu/error/output/kp/ki/kd/speed 均为长度 2 或 3 的序列：
      imu   -> 3 个欧拉角
      其余  -> 2 个电机
    """
    values = [
        float(imu[0]), float(imu[1]), float(imu[2]),
        float(error[0]), float(output[0]),
        float(error[1]), float(output[1]),
        float(kp[0]), float(ki[0]), float(kd[0]),
        float(kp[1]), float(ki[1]), float(kd[1]),
        float(speed[0]), float(speed[1]),
    ]
    data = _FRAME_STRUCT.pack(*values, int(frame_end) & 0xFFFFFFFF)
    assert len(data) == FRAME_SIZE, len(data)
    return data


def euler_from_quaternion(x, y, z, w):
    """四元数 -> (roll, pitch, yaw)，单位：弧度，ZYX 顺序。"""
    import math
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw
