# 差速底盘驱动(动力+imu)

> DD_Chassis_001 驱动板及其MCU固件包含板载imu和两个减速直流电机定速驱动，不包括激光雷达

## 驱动设计说明

- DD_Chassis_001的MCU固件以120Hz左右上报数据，帧格式见下方[串口数据帧格式](#串口数据帧格式)
- 本驱动包应该实现将MCU固件上传的信息解析到nav2里程计系统需要的话题，包含两轮速度和IMU偏航角
- 本驱动包应该根据配置文件配置一下内容
    - 使用的串口设备和波特率
    - 选择使用的IMU欧拉角数据序号(0~2)
    - 两轮速度物理值与固件数据值的换算系数(标定)
    - 其它有必要配置的项

## 串口数据帧格式

| 序号 | 数据宽度 | 数据类型 | name | 含义 |
| --- | --- | --- | --- | --- |
| 0 | 32bit | float | imu1 | imu欧拉角 |
| 1 | 32bit | float | imu2 | imu欧拉角 |
| 2 | 32bit | float | imu3 | imu欧拉角 |
| 3 | 32bit | float | error0 | 电机0的速度误差 |
| 4 | 32bit | float | output0 | 电机0的pid输出 |
| 5 | 32bit | float | error1 | 电机1的速度误差 |
| 6 | 32bit | float | output1 | 电机1的pid输出 |
| 7 | 32bit | float | 0Kp | 电机0的pid的Kp参数 |
| 8 | 32bit | float | 0Ki | 电机0的pid的Ki参数 |
| 9 | 32bit | float | 0Kd | 电机0的pid的Kd参数 |
| 10 | 32bit | float | 1Ki | 电机1的pid的Kp参数 |
| 11 | 32bit | float | 1Ki | 电机1的pid的Ki参数 |
| 12 | 32bit | float | 1Ki | 电机1的pid的Kd参数 |
| 13 | 32bit | float | speed0 | 电机0的速度 |
| 14 | 32bit | float | speed1 | 电机1的速度 |
| 15 | 32bit | uint32_t | end | 帧尾 |

## 命令格式

### 命令说明

- 所有串口命令都要带换行符'\n'
- 命令数值的实际物理意义请不要在意，详见[驱动设计说明](#驱动设计说明)。

### 固件命令处理函数注释

```c
/*!
 * @brief sz_at_qei_bdc_sc 组件的命令行接口
 *      接收命令格式：
 *          | SZ | qeiM    | <obj_index> | < item2set > | <value>  |
 *          | -- | argv[0] |  argv[1]    |      argv[2] |  argv[3] | 
 * 
 *      <obj_index>：实例索引，从"0"开始，有效范围 0~(sz_at_qei_bdc_sc_list_size-1)。
 *      <item2set> ：设置项，严格区分大小写，取值为下表中的字符串：
 *                   字符串 | 序号 | 含义
 *                   "Kp"   | (1)  | 控制器比例系数 Kp
 *                   "Ki"   | (2)  | 控制器积分系数(内部以 Ki*Ts 保存)
 *                   "Kd"   | (3)  | 控制器微分系数 Kd
 *                   "T"    | (4)  | 速度目标 Target
 *                   "N"    | (5)  | 编码器使用历史数量 use_history_num
 *      <value>    ：数值，"Kp/Ki/Kd/T"时应为浮点数，"N"时，应为大于"0"的正整数，输入为浮点时应忽略小数部分。
 *                   "N"时，大于 CFG_SZ_AT_QEI_BDC_SC_HISTORY_NUM_MAX 的输入会被钳位到该最大值，
 *                   并清零编码器历史相关变量，避免缩小历史窗口后残留项污染误差计算。
 * 
 * @param argc 命令行参数数量(包含 argv[0] 程序名)
 * @param argv 以 NULL 结尾的参数字符串数组，argv[0] 为程序名
 * @return int 返回值 0 表示成功，非零表示失败(具体见 sz_at_qei_bdc_sc_cli_error_t)
 */
```

### 使用命令范围

除了此处给出的需要实现的命令，其它都是调试命令，本驱动包不予实现。

速度指令：
```txt
sz qeiM <i> T <value>
```

### 命令示例

设置电机0速度为10
```txt
sz qeiM 0 T 10.0
```

设置电机1速度为-20.5
```txt
sz qeiM 1 T -20.5
```


## 本机实现（ros2_control 硬件组件）

真机接口做成 **1 个 System 组件**（轮速与 IMU 同帧同串口，拆成两个组件就要共享同一个串口 fd，且 Humble 没有 `imu_sensor_broadcaster`）：

| 文件 | 职责 |
| --- | --- |
| `serial_port.{hpp,cpp}` | 纯 C++ 串口封装：raw 8N1 非阻塞开/读/写 |
| `chassis_protocol.{hpp,cpp}` | 64 字节帧解析（靠帧尾魔数对齐）、`sz qeiM <i> T <v>` 命令组包 |
| `dd_chassis_001_system.{hpp,cpp}` | ros2_control `SystemInterface`：2 轮 velocity 指令 + position/velocity 状态；内部线程发布 `/imu` |
| `config/dd_chassis_001.ros2_control.xacro` | 全部参数（串口、帧尾、标定、IMU 选择）的带注释配置片段 |
| `dd_chassis_001_hardware.xml` | pluginlib 导出，插件名 `dd_chassis_001_hardware/DDChassis001System` |

装配方式：把 `config/dd_chassis_001.ros2_control.xacro` 的 `<ros2_control>` 块替换掉 URDF 里仿真用的 `ign_ros2_control` 块，`controllers.yaml` / `ekf.yaml` 不用改（`/diff_drive_controller/odom` 与 `/imu` 名字保持一致）。

编译依赖：
```bash
sudo apt install ros-humble-ros2-control ros-humble-ros2-controllers
```
然后 `colcon build --packages-select DD_Chassis_001_driver`。

实现约定（需要与固件确认/适配时改这里）：
- 上报帧按**小端**解析（`chassis_protocol.cpp` 的 `read_le_u32/read_le_f32`）。
- 固件只上报轮速，`position` 由速度积分得到；IMU 只上报欧拉角，`angular_velocity.z` 由相邻帧 yaw 差分得到（已处理 ±180° 跳变）。
- 通信中断（`frame_timeout_ms` 内无有效帧）时上报零速度，并按 `timeout_stops_motors` 向固件下发 0。
- 速度指令只在变化超过 `command_epsilon` 时下发，固件保持上一次目标值。
