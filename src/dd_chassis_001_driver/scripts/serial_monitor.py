#!/usr/bin/env python3
"""
Raw serial self-check and speed_scale calibration helper.

用途：
  1) 确认 frame_end(帧尾魔数)、波特率、字段顺序是否正确；
  2) 手动下发速度指令；
  3) 采集固件速度值，辅助标定 speed_scale。

示例：
  ros2 run dd_chassis_001_driver serial_monitor.py --device /dev/ttyUSB1 --frame-end 0x7F800000
  ros2 run dd_chassis_001_driver serial_monitor.py --send "SZ qeiM 0 T 10"
  ros2 run dd_chassis_001_driver serial_monitor.py \
      --calibrate-motor 0 --target 10 --measured-rad-s 12.5
"""

import argparse
import struct
import sys
import time

FRAME_FLOAT_COUNT = 15
FRAME_SIZE = FRAME_FLOAT_COUNT * 4 + 4


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--device', default='/dev/ttyUSB1', help='串口设备')
    parser.add_argument('--baud', type=int, default=1000000, help='波特率')
    parser.add_argument(
        '--frame-end', default='0x7F800000', help='帧尾魔数，与 xacro 的 frame_end 一致')
    parser.add_argument('--send', help='只发送一条命令后退出，如 "SZ qeiM 0 T 10"')
    parser.add_argument(
        '--calibrate-motor', type=int, choices=[0, 1], help='标定该电机：先下发 --target 再统计 speed 均值')
    parser.add_argument('--target', type=float, default=10.0, help='标定时下发的固件目标值')
    parser.add_argument('--measured-rad-s', type=float, help='实测轮子角速度(rad/s)，用于反算 speed_scale')
    parser.add_argument('--seconds', type=float, default=2.0, help='标定采样时长')
    args = parser.parse_args()

    try:
        import serial
    except ImportError:
        sys.exit('缺少 pyserial：sudo apt install python3-serial')

    end_bytes = struct.pack('<I', int(args.frame_end, 0) & 0xFFFFFFFF)

    try:
        port = serial.Serial(args.device, args.baud, timeout=0.05)
    except Exception as exc:  # noqa: BLE001
        sys.exit(f'打开串口失败 {args.device}: {exc}')

    def send(text):
        port.write((text + '\n').encode())
        print(f'>> {text}')

    if args.send:
        send(args.send)
        time.sleep(0.2)
        return

    calibrating = args.calibrate_motor is not None
    if calibrating:
        send(f'SZ qeiM {args.calibrate_motor} T {args.target}')
        print(f'采样 {args.seconds}s ...')

    buf = b''
    frames = 0
    speeds = {0: [], 1: []}
    last_print = 0.0
    t_start = time.time()
    last_report = time.time()
    latest = None

    while True:
        buf += port.read(256)

        # 在缓冲区里找帧尾魔数；魔数前 64 字节即一帧（协议无帧头）
        while True:
            i = buf.find(end_bytes)
            if i < 0:
                buf = buf[-(FRAME_SIZE - 1):]   # 保留可能是半帧的尾部
                break
            if i + 4 < FRAME_SIZE:
                buf = buf[i + 4:]              # 残帧，丢掉到魔数之后
                continue
            start = i + 4 - FRAME_SIZE
            fields = struct.unpack_from('<15f', buf, start)
            buf = buf[i + 4:]
            latest = fields
            frames += 1
            speeds[0].append(fields[13])
            speeds[1].append(fields[14])

        now = time.time()

        if calibrating and now - t_start >= args.seconds:
            values = speeds[args.calibrate_motor]
            if not values:
                print('没有收到任何有效帧，先排查 frame_end / 波特率')
            else:
                avg = sum(values) / len(values)
                print(f'固件目标={args.target} 上报 speed 平均值={avg:.4f}')
                if args.measured_rad_s:
                    # speed_scale 定义：关节角速度(rad/s) = 固件速度值 * speed_scale
                    print(
                        f'建议 speed_scale = measured_rad_s / speed_avg = '
                        f'{args.measured_rad_s / avg:.6f}')
            send(f'SZ qeiM {args.calibrate_motor} T 0')
            return

        if now - last_print > 0.5:
            last_print = now
            if latest is None:
                print('... 收不到有效帧')
                continue
            fps = frames / max(1e-6, now - last_report) if frames else 0.0
            print(
                f'帧率≈{fps:5.1f}Hz  '
                f'imu1={latest[0]:8.3f} imu2={latest[1]:8.3f} imu3={latest[2]:8.3f}  '
                f'speed0={latest[13]:8.3f} speed1={latest[14]:8.3f}  '
                f'err0={latest[3]:7.3f} out0={latest[4]:8.3f} err1={latest[5]:7.3f} '
                f'out1={latest[6]:8.3f}')
            frames = 0
            last_report = now


if __name__ == '__main__':
    main()
