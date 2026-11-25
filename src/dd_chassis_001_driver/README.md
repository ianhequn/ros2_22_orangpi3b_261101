# dd_chassis_001_driver

DD_Chassis_001 两轮差速底盘（含板载 IMU）的 ros2_control **System** 硬件组件。
MCU 以 ~120Hz 上报 64 字节帧（IMU 欧拉角 + 两轮速度），同时随时接收 `SZ qeiM <i> T <v>` 速度指令。

## 文件结构

| 路径 | 用途 |
| --- | --- |
| `include/.../config.hpp` + `src/config.cpp` | URDF `<param>` → `Config` 强类型解析与校验（纯 C++，可单测） |
| `include/.../serial_port.hpp` + `src/serial_port.cpp` | 串口：raw 8N1、非阻塞、波特率映射表 |
| `include/.../chassis_protocol.hpp` + `src/chassis_protocol.cpp` | 64 字节帧解析（帧尾魔数对齐）+ `SZ qeiM` 命令组包 |
| `include/.../data_exchange.hpp` | 线程间数据面：状态快照、命令槽、新帧事件 |
| `include/.../dd_chassis_001_system.hpp` + `src/dd_chassis_001_system.cpp` | ros2_control 组件本体 + IO 线程 + IMU 线程 |
| `dd_chassis_001_driver.xml` | pluginlib 描述（插件名 `dd_chassis_001_driver/DDChassis001System`） |
| `config/dd_chassis_001.ros2_control.xacro` | **全部硬件参数**（串口/帧尾/标定/IMU），逐项注释 |
| `config/controllers.yaml` | 验证用控制器（joint_state_broadcaster + diff_drive_controller） |
| `launch/chassis_test.launch.py` | 验证用启动 |
| `scripts/serial_monitor.py` | 裸串口自检 / 标定辅助（不依赖 ros2_control） |
| `scripts/chassis_monitor.py` | 话题打表（轮速/yaw/odom 单行刷新） |
| `scripts/qos_relay.py` | Twist QoS 中继（RELIABLE→BEST_EFFORT），键盘遥控必备 |
| `test/` | 参数、协议、数据面、插件加载的单元测试 |
| `note/` | `设计说明.md`（架构）、`TODO.md`（进度与验证）、`教程.md`（官方文档译文） |

## 线程模型

```
IO 线程（独占串口，只阻塞在 poll）
  ├─ poll(串口, eventfd) → 解析帧 → 双缓冲快照 → 积分位置 → 判定超时
  └─ 命令槽 → 组包 → 写串口
IMU 线程（只碰 DDS）：等新帧事件 → 发布 /imu（时间戳=帧到达时刻）
控制器回调 read()/write()：只做内存读写，不做任何 syscall
```

## 接口

| 接口 | 方向 | 单位 |
| --- | --- | --- |
| `<left>/velocity`, `<right>/velocity` | command | rad/s |
| `<left>/position`, `<right>/position` | state | rad |
| `<left>/velocity`, `<right>/velocity` | state | rad/s |

另发布 `/imu`（sensor_msgs/Imu）：仅 yaw 与 yaw 角速度可信，加速度不提供。

## 编译与测试

```bash
cd ~/MyWork/Robot/ros2/ws_orangepi_01
colcon build --symlink-install --packages-select dd_chassis_001_driver
colcon test --packages-select dd_chassis_001_driver && colcon test-result --verbose
```

## 装配到机器人

把 `config/dd_chassis_001.ros2_control.xacro` 的 `<ros2_control>` 块替换掉 URDF 里仿真用的
`ign_ros2_control` 块；`controllers.yaml` / `ekf.yaml` 不需要改（`/diff_drive_controller/odom`
与 `/imu` 名字保持一致）。

**注意**：`diff_drive_controller` 的命令订阅是 `BEST_EFFORT`，nav2/遥控若用默认 RELIABLE 发布，
消息会被 DDS 静默丢弃。此时需在发布端设置 `qos_reliability: best_effort`，或加中继。

详细设计与验证步骤见 `note/` 下的三份文档。
