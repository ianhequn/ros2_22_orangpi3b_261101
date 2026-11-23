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
| 1 | 参数层 | ✅ | `on_init` 解析全部 param + 校验 | 故意填错参数 → 明确报错且拒绝加载 |
| 2 | 串口类 | ✅ | `serial_port.{hpp,cpp}` | gtest/小程序能读到 120Hz 原始字节 |
| 3 | 协议类 | ✅ | `chassis_protocol.{hpp,cpp}` | 4 个 gtest 用例全过 |
| 4 | 数据面 | ✅ | 快照双缓冲 + 命令槽 + 新帧事件 | `test_data_exchange` 6 例通过 |
| 5 | IO 线程 | ✅ | eventfd + poll + 帧解析 + 积分 | `debug_log` 帧率≈120Hz，手动轮子 speed 变化 |
| 6 | 状态接口 | ✅ | `read()` 只拷快照 | `/joint_states` 频率=update_rate、数值跟随 |
| 7 | 命令链路 | ✅ | `write()` → 命令槽 → IO 线程下发 | 轮子转；打印出真实 `SZ qeiM` 字符串 |
| 8 | 标定 | ⬜ | `speed_scale` / `invert` 定值（待实机） | 前进方向正确；1m 直线 → odom x≈1m |
| 9 | IMU 线程 | ✅ | 事件驱动 + 抽帧 + 时间戳偏移 | `hz /imu`≈120Hz、yaw 方向正确 |
| 10 | 安全与生命周期 | ✅ | 超时/停机优先/线程 join | 拔串口→告警+归零+停机且能正常退出 |

状态标记：⬜ 未开始 / 🟨 进行中 / ✅ 完成 / ⛔ 阻塞（写明原因）

---

## 0. 骨架与 lint 清理

- [x] 确认 `dd_chassis_001_system.hpp/.cpp` 骨架能编译、`test_load_dd_chassis_001_system` 通过
- [x] `uncrustify`：用 `ament_uncrustify --reformat` 修正格式（2 空格缩进等），并顺手修正被误改的 doxygen 注释块
- [x] `xmllint`：真因是包名含**大写**，违反 ROS 包名规范 `[a-z](_?[a-z0-9]+)*`
- [x] **包已迁移为小写 `dd_chassis_001_driver`** → 根因消除，`xmllint` 恢复启用并通过
- [x] `copyright`（可选）：沿用原模板的 `set(ament_cmake_copyright_FOUND TRUE)`，不补头注释
- [x] 旧包（大写）已删除，`build/ install/ log/` 已清空重建

**完成标准**：`colcon build --symlink-install` + `colcon test` 全绿（17 tests, 0 failures）。

> 注意 1：改 `CMakeLists.txt` 里的 lint 开关后必须 `rm -rf build/<pkg> install/<pkg>` 再构建，
> 否则 `CMakeCache` 里缓存的 `ament_cmake_*_FOUND` 变量会继续生效。
>
> 注意 2：包名迁移需同步 7 处（本次已全部改完）：
> `project()`/`add_library()`、xml 的 `<library path>` 与 `<class name>`、
> `PLUGINLIB_EXPORT_CLASS` 的命名空间、`#include "<包名>/..."`、`include/<包名>/` 目录、
> 头文件守卫宏、xacro 里的 `<plugin>`。
> 迁移后务必 `rm -rf build install log`，否则 ament 索引会残留旧插件名。

---

## 1. 参数层（`on_init`）

- [x] 参数集中为 `struct Config`（`include/.../config.hpp`），解析逻辑独立成 `parse_config()`（`src/config.cpp`），不依赖 ROS，可单测
- [x] 按 `设计说明.md` §6 清单逐项解析 `info.hardware_parameters`
- [x] 类型校验：整串消费（拒绝 `1.0abc`）、`speed_scale>0`、`imu_yaw_field∈{1,2,3}`、`imu_yaw_sign∈{1,-1}`、`frame_end≠0`、各超时/限幅非负
- [x] 区分两类失败：`kMissing`（必填缺失）与 `kInvalid`（值非法），日志中带上参数名
- [x] 交叉校验：左右轮 joint 名不同、`motor_index` 不同
- [x] 校验 `info.joints` 中确实存在 `left/right_wheel_joint`；成功后打印一行汇总配置
- [x] 单测 `test/test_config.cpp`（8 例）：完整解析、缺失、非法、尾随垃圾、零系数、越界、重名/重电机号、默认值

**完成标准**：`colcon test` 全绿（32 tests, 0 failures）；URDF 填错参数时启动即报错并拒绝加载。

> 测试发现并修复的真实缺陷：`left/right_motor_index` 默认值都是 0，
> 导致"不显式配置就必然被交叉校验拦下"。现 `Config()` 构造函数中把 `right.motor_index` 默认设为 1。

---

## 2. 串口类（`serial_port.{hpp,cpp}`）

- [x] `open()`：`O_RDWR|O_NOCTTY|O_NONBLOCK|O_CLOEXEC` + `cfmakeraw` + 8N1 + 关闭硬件流控
- [x] 波特率映射表补全（1200~4000000，含 **1000000**），未知值显式失败
- [x] `read()`：`EAGAIN/EWOULDBLOCK/EINTR` → 返回 0，不当作错误
- [x] `write()`：处理 `EINTR`，`EAGAIN` 有限重试，坏 fd 返回 false
- [x] 暴露 `fd()` 供 `poll()`；RAII 析构关 fd；禁拷贝

**验证**：编译期类型检查 + 第 4 步实机跑通（120Hz 帧正常解析）。

---

## 3. 协议类（`chassis_protocol.{hpp,cpp}`）

- [x] 小端 `read_le_u32` / `read_le_f32`（不依赖主机端序）
- [x] `push()` 循环解析，一次可解出多帧
- [x] 找不到魔数时**保留末尾 63 字节**
- [x] 魔数出现在不足 64 字节处 → 丢掉到魔数之后并**继续扫**
- [x] 命令组包 `SZ qeiM <i> T <v>\n`（前缀大写）
- [x] 字段索引用 `enum FieldIndex` 常量
- [x] `reset()` 供重开串口时清残留半帧

**验证**：`test/test_chassis_protocol.cpp` 6 例全过（逐字节、前置垃圾、丢 5 字节恢复、多帧连发、半帧等待、命令格式）。

---

## 4. 数据面（`data_exchange.hpp`）

- [x] `StateSnapshot`：两轮速度/位置、yaw、yaw_rate、帧到达时刻、valid/comm_alive
- [x] `SnapshotBuffer`：双缓冲 + `atomic` 索引，读者整块拷贝无锁
- [x] `CommandSlot`：SPSC 覆盖式 + `priority` 标志 + 互斥保护
- [x] `FrameSignal`：序号 + 条件变量，`wait_for_new()` 不重复发旧帧

**验证**：`test/test_data_exchange.cpp` 6 例全过（发布/读取、覆盖、优先级、超时、唤醒）。

---

## 5. IO 线程

- [x] `on_configure`：开串口 + 建 eventfd + 起 IO 线程；`on_cleanup`：置标志 → 唤醒 → join → 关 fd
- [x] `poll()` 同时监听串口 fd 与 eventfd，超时 100ms（保证能及时看到停止标志）
- [x] 读到的字节全部喂给协议类，解出帧则 `apply_frame()`
- [x] 位置积分在 IO 线程用**真实帧间隔**完成
- [x] `comm_alive` 由 IO 线程按 `frame_timeout_ms` 判定；断线时上报 0、按配置 `send_stop()`、限流告警
- [x] `debug_log` 打印原始 speed / 换算后速度 / 位置 / yaw

**验证**：`debug_log=true` 时每秒一行、帧率≈120Hz；拔串口出现超时告警。

---

## 6. 状态接口

- [x] `read()` 只取快照并拷贝到 `wheel_velocity_/wheel_position_`，**无 syscall、无锁等待**
- [x] `comm_alive=false` 时上报 0，但**不返回 ERROR**

**验证**：`/joint_states` 频率 = `update_rate`，手转轮子数值跟随。

---

## 7. 命令链路

- [x] `write()`：`isfinite` 过滤 → 限幅 → 与原值比较（`command_epsilon`）→ 填命令槽
- [x] 填槽后 `eventfd` 唤醒 IO 线程（指令延迟降到微秒级）
- [x] IO 线程取槽 → `SERIAL` 换算（除法 + `invert`）→ 组包写出
- [x] `command_repeat_period_ms` 支持周期重发
- [x] `on_deactivate` 填 `{0,0}` 并带 `priority`，不被去重吞掉

**验证**：发 BEST_EFFORT 的 `cmd_vel_unstamped` 后轮子转；`debug_log` 可见下发值。

---

## 8. 标定（待实机）

- [ ] 用 `serial_monitor.py --calibrate-motor N --target V --measured-rad-s X` 得到 `speed_scale`
- [ ] 写入 `config/dd_chassis_001.ros2_control.xacro` 的 `left/right_speed_scale`
- [ ] 方向不符时改 `left/right_invert`（scale 恒为正）

**待实机完成**（工具已就绪）。

---

## 9. IMU 线程


- [x] `on_activate` 起 IMU 线程，`on_deactivate` 停并 join
- [x] 等 `frame_seq` 变化（条件变量），非定时器
- [x] `imu_publish_rate` 作抽帧上限
- [x] 时间戳取**帧到达时刻**，开机记 `ros_now - steady_now` 偏移，按 `imu_stamp_offset_calib_s` 周期校正
- [x] 协方差：yaw/yaw_rate 可信，其余 1e6；`linear_acceleration_covariance[0] = -1`
- [x] QoS `best_effort`，depth 10

**验证**：`ros2 topic hz /imu` ≈ 配置频率；yaw 方向与 REP-103 一致。

---

## 10. 安全与生命周期

- [x] `comm_alive` 由 IO 线程维护（毫秒级，不受控制周期影响）
- [x] 超时：IO 线程下发 0 + 快照归零 + 限流告警
- [x] `on_deactivate` 填 0 命令（priority）并唤醒 IO；`on_shutdown` / `on_error` 允许直写一次 0
- [x] 退出顺序：置停止标志 → `wake_io()` / `wake_all()` → join
- [x] 析构函数兜底 join，避免异常路径泄漏线程

**验证**：拔串口 → 告警 + 归零 + 停机；Ctrl-C 后无卡死。

---

## 阻塞与依赖

| 依赖项 | 需要的步骤 | 现状 |
| --- | --- | --- |
| `frame_end` 真值 | 2、3、4 | 已填 `0x7F800000`（待确认） |
| 波特率 | 2 | 已填 `1000000` |
| `speed_scale` 初值 | 6、7 | 仍为 `1.0`，第 6 步"轮子不转"时先加大指令值再怀疑代码 |
| 电机号与左右轮对应 | 6 | 未确认 |
| IMU 三轴哪一路是 yaw | 8 | 已知 `imu_yaw_field=3`（待复核方向） |
