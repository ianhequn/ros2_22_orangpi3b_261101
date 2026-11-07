#pragma once

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <hardware_interface/system_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include "DD_Chassis_001_driver/chassis_protocol.hpp"
#include "DD_Chassis_001_driver/serial_port.hpp"

namespace dd_chassis_001
{
/// 每个轮子的配置（全部来自 URDF <hardware><param>）
struct WheelConfig
{
  std::string joint_name;      // 对应 URDF 中的 joint 名
  int motor_index = 0;         // 固件里的电机序号（0 或 1）
  double speed_scale = 1.0;    // 固件速度值 → 关节角速度(rad/s) 的换算系数
  bool invert = false;         // 关节正方向与固件正方向相反时置 true
};

struct DriverConfig
{
  std::string serial_device = "/dev/ttyUSB0";
  int baud_rate = 115200;
  uint32_t frame_end = 0;             // 帧尾魔数，用于在字节流中对齐 64 字节帧
  int frame_timeout_ms = 500;         // 超过该时间收不到有效帧即判定通信中断
  bool timeout_stops_motors = true;   // 通信中断时向固件发 0 速度（安全默认）
  double max_wheel_velocity = 25.0;   // 关节角速度限幅(rad/s)，<=0 表示不限幅
  double command_epsilon = 0.01;      // 指令变化小于该值不下发，避免刷串口

  bool publish_imu = true;
  std::string imu_topic = "imu";
  std::string imu_frame_id = "imu_link";
  int imu_yaw_field = 1;              // 用 imu1/imu2/imu3 中的哪一路做 yaw（1~3）
  bool imu_angle_is_deg = true;       // 固件欧拉角单位是否为度
  double imu_yaw_sign = 1.0;          // yaw 取反用（-1 表示反向）
  double imu_publish_rate = 100.0;    // IMU 发布频率(Hz)
};

/// ros2_control 系统组件：两轮差速电机 + 板载 IMU，同一个串口、同一帧数据。
class DDChassis001System : public hardware_interface::SystemInterface
{
public:
  ~DDChassis001System() override;

  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  using SteadyClock = std::chrono::steady_clock;

  /// 向固件下发一个轮子的目标速度（关节角速度 rad/s）
  void send_wheel_command(size_t wheel, double joint_velocity);
  /// 处理一帧上报数据：更新轮速/里程计数/IMU 快照
  void update_from_frame(const ChassisFrame & frame);
  /// IMU 发布线程
  void imu_publish_loop();

  DriverConfig cfg_;
  std::vector<WheelConfig> wheels_;

  SerialPort port_;
  ChassisProtocol protocol_{cfg_.frame_end};

  // ros2_control 接口数据（顺序与 wheels_ 一致）
  std::vector<double> hw_position_;
  std::vector<double> hw_velocity_;
  std::vector<double> hw_velocity_cmd_;
  std::vector<double> last_sent_cmd_;
  std::vector<bool> has_sent_cmd_;

  // 通信状态
  SteadyClock::time_point last_frame_time_{};
  bool got_frame_ = false;
  bool comm_alive_ = false;   // 由 read() 更新，write() 据此决定是否强制停机

  // IMU 快照（read() 写，发布线程读）
  std::mutex imu_mutex_;
  double imu_yaw_rad_ = 0.0;
  double imu_yaw_rate_ = 0.0;
  bool imu_valid_ = false;
  SteadyClock::time_point last_imu_time_{};

  // IMU 发布
  std::shared_ptr<rclcpp::Node> imu_node_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  std::thread imu_thread_;
  std::atomic<bool> imu_thread_running_{false};
};

}  // namespace dd_chassis_001
