# 开发 TODO

配套文档：`设计说明.md`（架构与契约）、`教程.md`（框架用法）。

## 推进原则

1. **自底向上**：纯 C++ → 线程 → ros2_control 接口 → 标定 → IMU → 安全。
2. **每步一个可观测信号**：能跑一条命令、看一个数字变化，不接受"这步写完了"。
3. **每步结束必须 `colcon build` + `colcon test` 通过**，不留半成品。
4. **不引入新依赖**（调试脚本属验证工具，可选）。

## 进度总表

| # | 步骤 | 状态 | 产出 | 验证信号 |
| --- | --- | --- | --- | --- |
| 0 | 骨架与 lint 清理 | ✅ | 能编能测、lint 静音 | `colcon build && colcon test` 全绿 |
| 1 | 参数层 | ⬜ | `on_init` 解析全部 param + 校验 | 故意填错参数 → 明确报错且拒绝加载 |
| 2 | 串口类 | ⬜ | `serial_port.{hpp,cpp}` | gtest/小程序能读到 120Hz 原始字节 |
| 3 | 协议类 | ⬜ | `chassis_protocol.{hpp,cpp}` | 4 个 gtest 用例全过 |
| 4 | IO 线程 | ⬜ | eventfd + poll + 双缓冲快照 | `debug_log` 帧率≈120Hz，手动轮子 speed 变化 |
| 5 | 状态接口 | ⬜ | `read()` 拷快照 + 位置积分 | `/joint_states` 频率=update_rate、数值跟随 |
| 6 | 命令链路 | ⬜ | 命令槽 + `write()` + eventfd 唤醒 | 轮子转；打印出真实 `SZ qeiM` 字符串 |
| 7 | 标定 | ⬜ | `speed_scale` / `invert` 定值 | 前进方向正确；1m 直线 → odom x≈1m |
| 8 | IMU 线程 | ⬜ | `frame_seq` + cv + 时间戳偏移 | `hz /imu`≈120Hz、yaw 方向正确 |
| 9 | 安全收尾 | ⬜ | 超时/停机优先/生命周期 join | 拔串口 → 告警+归零+停机且能正常退出 |

状态标记：⬜ 未开始 / 🟨 进行中 / ✅ 完成 / ⛔ 阻塞（写明原因）

---

## 0. 骨架与 lint 清理

- [x] 确认 `dd_chassis_001_system.hpp/.cpp` 骨架能编译、`test_load_dd_chassis_001_system` 通过
- [x] `uncrustify`：用 `ament_uncrustify --reformat` 修正格式（2 空格缩进等），并顺手修正被误改的 doxygen 注释块
- [x] `xmllint`：**真实原因是包名 `DD_Chassis_001_driver` 含大写，不符合 ROS 规范 `[a-z](_?[a-z0-9]+)*`**，无法靠改代码解决 → 在 CMakeLists 里 `set(ament_cmake_xmllint_FOUND TRUE)` 禁用
- [x] `copyright`（可选）：沿用原模板的 `set(ament_cmake_copyright_FOUND TRUE)`，不补头注释

**完成标准**：`colcon build --symlink-install` + `colcon test` 全绿（14 tests, 0 failures）。

> 注意：改 `CMakeLists.txt` 里的 lint 开关后必须 `rm -rf build/<pkg> install/<pkg>` 再构建，
> 否则 `CMakeCache` 里缓存的 `ament_cmake_*_FOUND` 变量会继续生效。

---

## 1. 参数层（`on_init`）

- [ ] 按 `设计说明.md` §6 的清单逐项解析 `info.hardware_parameters`
- [ ] 类型校验：`baud_rate`/`frame_timeout_ms` 整数、`speed_scale` 浮点且 ≠0、`imu_yaw_field` ∈ {1,2,3}
- [ ] 必填项缺失或非法 → `RCLCPP_ERROR` + 返回 `ERROR`（不要静默取默认值）
- [ ] 校验 `info.joints` 中确实存在 `left/right_wheel_joint`
- [ ] 参数集中成一个 `struct Config`，供后续模块只读访问

**验证**：URDF 里故意删掉 `frame_end` / 把 `baud_rate` 写成 `abc`，启动后应明确报错、控制器不启动。

---

## 2. 串口类（`serial_port.{hpp,cpp}`）

- [ ] `open()`：`O_RDWR|O_NOCTTY|O_NONBLOCK|O_CLOEXEC` + `cfmakeraw` + 8N1 + 关闭流控
- [ ] 波特率映射表补全（含 1000000 等高速档），未知值返回明确错误
- [ ] `read()` 非阻塞：`EAGAIN` → 返回 0，不视为错误
- [ ] `write()` 保证一条命令完整写出（处理 `EAGAIN`/`EINTR`）
- [ ] 暴露 `fd()` 供 `poll()` 使用
- [ ] RAII：析构关 fd，禁拷贝

**验证**：gtest 或用例程序，接真串口持续读 1s，字节数 ≈ 波特率/10。

---

## 3. 协议类（`chassis_protocol.{hpp,cpp}`）

- [ ] 小端 `read_le_u32` / `read_le_f32`（不依赖主机端序）
- [ ] `push()` 追加字节并**循环**解析出所有完整帧
- [ ] 找不到魔数时**保留末尾 63 字节**
- [ ] 魔数位于不足 64 字节处 → 丢掉到魔数之后并**继续扫**
- [ ] 命令组包 `SZ qeiM <i> T <v>\n`（前缀大写）
- [ ] 字段索引用 `enum` 常量，不用魔法数字

**验证**（gtest 四例）：
1. 完整帧逐字节喂入 → 正确解出
2. 前置随机垃圾字节 → 正确解出
3. 中间丢 5 字节 + 后续完整帧 → 能解出后续帧
4. 三帧连发一次 push → 循环解出多帧，最后一帧值正确

---

## 4. IO 线程

- [ ] `on_configure` 开串口 + 建 eventfd + 起线程；`on_cleanup` 置停止标志 → 唤醒 → join → 关 fd
- [ ] `poll()` 同时监听串口 fd 与 eventfd
- [ ] 读到的字节全部喂给协议类，解出帧则更新「最新快照」
- [ ] 快照内容：两轮 speed 原始值、两轮 speed 换算值、累计位置、yaw、帧到达 `steady_clock`
- [ ] 位置积分在 IO 线程做（它知道真实帧间隔）
- [ ] 双缓冲 + `atomic<uint32_t>` 索引发布快照
- [ ] `debug_log`：打印帧率 + 原始 speed + 换算后值

**验证**：启动后日志帧率≈120Hz；手转轮子 speed 变化；`debug_log=false` 时无日志输出。

---

## 5. 状态接口

- [ ] `read()` 只做：取快照索引 → 拷贝到 `wheel_velocity_[2]` / `wheel_position_[2]`
- [ ] 快照超时 → 上报 0（**不返回 ERROR**）
- [ ] `read()` 内禁止 syscall / 锁等待 / 日志 IO

**验证**：`ros2 topic hz /joint_states` ≈ `update_rate`；手转轮子 velocity 变化且方向为正（向前）。

---

## 6. 命令链路

- [ ] `write()`：限幅 → 单位换算 → 与上次值比较（`command_epsilon`）→ 填命令槽
- [ ] 命令槽：SPSC 覆盖式 + `atomic` + `priority` 标志（0 速度须插队，不被去重吞掉）
- [ ] 填槽后 `write(eventfd, 1)` 唤醒 IO 线程
- [ ] IO 线程取槽 → 组包 → 写串口
- [ ] `command_repeat_period_ms`：>0 时按周期重发

**验证**：发 `/diff_drive_controller/cmd_vel_unstamped`（**BEST_EFFORT**）→ 轮子转；`debug_log` 打印 `"SZ qeiM 0 T xx\n"`；测填槽→写出延迟 < 2ms。

---

## 7. 标定

- [ ] 固定固件目标值，采样平均上报 speed
- [ ] 实测轮子实际 rad/s（转 N 圈计时）
- [ ] `speed_scale = 实测 rad/s ÷ 上报 speed`，写入 URDF 参数
- [ ] 方向不符 → `left/right_invert`（scale 恒为正）

**验证**：前进指令使两轮同向正转；直线走 1m，`odom` x 变化 ≈1m（误差 <5%）。

---

## 8. IMU 线程

- [ ] `on_activate` 起 IMU 线程，`on_deactivate` 停
- [ ] 等 `frame_seq` 变化（`condition_variable`），非定时器
- [ ] `imu_publish_rate` 作抽帧上限
- [ ] 时间戳用**帧到达时刻**；开机记 `ros_now - steady_now` 偏移，约 10s 校正一次
- [ ] 协方差：yaw/yaw_rate 可信，其余置 1e6；`linear_acceleration_covariance[0] = -1`
- [ ] QoS：`BEST_EFFORT`，depth 10

**验证**：`ros2 topic hz /imu` ≈ 上报频率且无累积漏发；车体逆时针转 → yaw 增大；EKF 不发散。

---

## 9. 安全收尾

- [ ] `comm_alive` 由 IO 线程维护（毫秒级判定，不受控制周期影响）
- [ ] 超时：IO 线程下发 0 + `read()` 上报 0 + 限流告警
- [ ] `on_deactivate` / `on_shutdown` / `on_error`：填 0 命令 + 唤醒 IO；`on_shutdown` 带超时兜底可直写一次
- [ ] 退出顺序：置标志 → 唤醒所有等待者 → join
- [ ] 关闭 `debug_log` 默认值

**验证**：运行中拔串口 → 告警 + 归零 + 电机停；`Ctrl-C` 后无卡死、线程全部退出。

---

## 阻塞与依赖

| 依赖项 | 需要的步骤 | 现状 |
| --- | --- | --- |
| `frame_end` 真值 | 2、3、4 | 已填 `0x7F800000`（待确认） |
| 波特率 | 2 | 已填 `1000000` |
| `speed_scale` 初值 | 6、7 | 仍为 `1.0`，第 6 步"轮子不转"时先加大指令值再怀疑代码 |
| 电机号与左右轮对应 | 6 | 未确认 |
| IMU 三轴哪一路是 yaw | 8 | 已知 `imu_yaw_field=3`（待复核方向） |
