/**
 * @file dd_chassis_001_system.hpp
 * @brief DD_Chassis_001 两轮差速底盘（含板载 IMU）的 ros2_control 硬件组件。
 *
 * 线程模型（详见 note/设计说明.md §2）：
 *  - IO 线程：独占串口 fd，poll 等数据/命令，解析帧、积分位置、下发命令；
 *  - IMU 线程：等新帧事件后发布 /imu，绝不碰串口；
 *  - 控制器回调 read()/write()：只做内存读写，不做任何 syscall。
 */
#ifndef DD_CHASSIS_001_DRIVER__DD_CHASSIS_001_SYSTEM_HPP_
#define DD_CHASSIS_001_DRIVER__DD_CHASSIS_001_SYSTEM_HPP_

#include <array>
#include <atomic>
#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "dd_chassis_001_driver/chassis_protocol.hpp"
#include "dd_chassis_001_driver/config.hpp"
#include "dd_chassis_001_driver/data_exchange.hpp"
#include "dd_chassis_001_driver/serial_port.hpp"
#include "dd_chassis_001_driver/visibility_control.hpp"

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "sensor_msgs/msg/imu.hpp"

namespace dd_chassis_001_driver
{

/**
 * @brief 两轮差速底盘系统组件。
 *
 * 对外接口（接口名为 "<joint>/<interface>"，joint 名必须与 URDF 一致）：
 *  | 接口                | 方向    | 单位  | 含义           |
 *  | ------------------- | ------- | ----- | -------------- |
 *  | <left>/velocity     | command | rad/s | 左轮目标角速度 |
 *  | <right>/velocity    | command | rad/s | 右轮目标角速度 |
 *  | <left>/position     | state   | rad   | 左轮累计转角   |
 *  | <left>/velocity     | state   | rad/s | 左轮实测角速度 |
 *  | <right>/position    | state   | rad   | 右轮累计转角   |
 *  | <right>/velocity    | state   | rad/s | 右轮实测角速度 |
 *
 * 另外发布 <imu_topic>（sensor_msgs/Imu），只保证 yaw 与 yaw 角速度可信。
 */
class DDChassis001System : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(DDChassis001System)

  DDChassis001System() = default;
  ~DDChassis001System() override;

  /**
 * @brief 解析参数并校验（不碰硬件）。
 * @return SUCCESS 参数合法；ERROR 则组件加载失败
 */
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  /// @brief 打开串口、建 eventfd、启动 IO 线程。
  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  /// @brief 停止 IO 线程并关闭串口。
  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  /// @brief 停机后释放资源。
  hardware_interface::CallbackReturn on_shutdown(
    const rclcpp_lifecycle::State & previous_state) override;

  /// @brief 启动 IMU 线程、清零状态。
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  /// @brief 停机并停止 IMU 线程。
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  /// @brief error 状态兜底：尽量让电机停下。
  hardware_interface::CallbackReturn on_error(
    const rclcpp_lifecycle::State & previous_state) override;

  /// @brief 导出状态接口（指针指向成员数组，必须固定大小）。
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  /// @brief 导出命令接口。
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  /**
 * @brief 周期读取：只拷贝快照到状态变量，不碰串口。
 * @param period 控制周期（用于插值/超时判断）
 */
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  /**
 * @brief 周期写：限幅 + 换算 + 去重后放入命令槽，不碰串口。
 */
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  using SteadyClock = std::chrono::steady_clock;
  static constexpr size_t kWheelCount = 2;

  /// @brief IO 线程主循环：poll 串口与 eventfd，解析帧、积分、下发命令。
  void io_loop();

  /// @brief IMU 线程主循环：等新帧事件后发布 /imu。
  void imu_loop();

  /**
 * @brief 把一帧原始数据换算成状态并写入快照（IO 线程内调用）。
 * @param frame 解析出的原始帧
 * @param now 帧到达时刻
 */
  void apply_frame(const ChassisFrame & frame, SteadyClock::time_point now);

  /**
 * @brief 组包并写出速度命令（IO 线程内调用）。
 * @param velocity 两轮目标角速度 [rad/s]
 */
  void send_velocity(const std::array<double, 2> & velocity);

  /// @brief 让两个轮子立即停机（可指定是否走直写路径）。
  void send_stop();

  /// @brief 唤醒 IO 线程（eventfd 写入）。
  void wake_io();

  /// @brief 把 steady_clock 时刻转换为 ROS 时间（用开机记录的偏移）。
  rclcpp::Time to_ros_time(SteadyClock::time_point tp) const;

  /// @brief 配置（on_init 解析，之后只读）。
  Config cfg_;

  // ---- 硬件与协议（只由 IO 线程使用）----
  SerialPort port_;
  ChassisProtocol protocol_{0};
  int event_fd_{-1};

  // ---- 线程 ----
  std::thread io_thread_;
  std::thread imu_thread_;
  std::atomic<bool> io_running_{false};
  std::atomic<bool> imu_running_{false};

  // ---- 数据面 ----
  SnapshotBuffer snapshot_;   ///< IO 线程 -> read() / IMU 线程
  CommandSlot command_slot_;  ///< write() -> IO 线程
  FrameSignal frame_signal_;  ///< IO 线程 -> IMU 线程

  // ---- IO 线程私有状态 ----
  SteadyClock::time_point last_frame_time_{};
  bool first_frame_{true};
  std::array<double, 2> last_sent_velocity_{0.0, 0.0};  ///< 已下发的目标值
  bool has_sent_{false};                                ///< 是否下发过
  SteadyClock::time_point last_send_time_{};
  bool logged_alive_{true};                             ///< 上次是否已打印"通信正常"

  // ---- IMU ----
  std::shared_ptr<rclcpp::Node> imu_node_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
  SteadyClock::time_point ros_time_offset_stamp_{};   ///< 上次校正偏移的时刻
  rclcpp::Duration ros_time_offset_{0, 0};            ///< ROS time - steady time

  // ---- 接口数据（指针被控制器持有，禁止重新分配）----
  double wheel_position_[kWheelCount] = {0.0, 0.0};
  double wheel_velocity_[kWheelCount] = {0.0, 0.0};
  double wheel_velocity_cmd_[kWheelCount] = {0.0, 0.0};

  /// @brief 限流日志用的固定时钟
  static rclcpp::Clock & throttle_clock();
};

}  // namespace dd_chassis_001_driver

#endif  // DD_CHASSIS_001_DRIVER__DD_CHASSIS_001_SYSTEM_HPP_
