#include "DD_Chassis_001_driver/dd_chassis_001_system.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <string>

#include "pluginlib/class_list_macros.hpp"

namespace dd_chassis_001
{
namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kTwoPi = 2.0 * kPi;

rclcpp::Logger logger()
{
  return rclcpp::get_logger("dd_chassis_001");
}

/// THROTTLE 宏需要一个 Clock，组件内没有 node 可借，用固定节拍的静态时钟
rclcpp::Clock & throttle_clock()
{
  static rclcpp::Clock clock(RCL_STEADY_TIME);
  return clock;
}

std::string param(
  const hardware_interface::HardwareInfo & info, const std::string & key,
  const std::string & fallback)
{
  const auto it = info.hardware_parameters.find(key);
  return it == info.hardware_parameters.end() ? fallback : it->second;
}

double param_double(
  const hardware_interface::HardwareInfo & info, const std::string & key, double fallback)
{
  const std::string text = param(info, key, "");
  if (text.empty()) {
    return fallback;
  }
  std::string::size_type used = 0;
  try {
    const double value = std::stod(text, &used);
    return used == text.size() ? value : fallback;
  } catch (const std::exception &) {
    RCLCPP_WARN(logger(), "参数 %s=\"%s\" 不是合法浮点数，使用默认值 %f",
      key.c_str(), text.c_str(), fallback);
    return fallback;
  }
}

int param_int(
  const hardware_interface::HardwareInfo & info, const std::string & key, int fallback)
{
  return static_cast<int>(param_double(info, key, static_cast<double>(fallback)));
}

bool param_bool(
  const hardware_interface::HardwareInfo & info, const std::string & key, bool fallback)
{
  const std::string text = param(info, key, "");
  if (text.empty()) {
    return fallback;
  }
  return text == "true" || text == "True" || text == "1";
}

/// 支持十进制与 0x 前缀的 uint32 解析（帧尾魔数用）
uint32_t param_u32(
  const hardware_interface::HardwareInfo & info, const std::string & key, uint32_t fallback)
{
  const std::string text = param(info, key, "");
  if (text.empty()) {
    return fallback;
  }
  try {
    return static_cast<uint32_t>(std::stoul(text, nullptr, 0));
  } catch (const std::exception &) {
    RCLCPP_WARN(logger(), "参数 %s=\"%s\" 不是合法整数，使用默认值 %u",
      key.c_str(), text.c_str(), fallback);
    return fallback;
  }
}

/// 把角度归一化到 (-pi, pi]
double wrap_angle(double angle)
{
  angle = std::fmod(angle, kTwoPi);
  if (angle > kPi) {
    angle -= kTwoPi;
  } else if (angle <= -kPi) {
    angle += kTwoPi;
  }
  return angle;
}
}  // namespace

DDChassis001System::~DDChassis001System()
{
  imu_thread_running_ = false;
  if (imu_thread_.joinable()) {
    imu_thread_.join();
  }
  port_.close();
}

hardware_interface::CallbackReturn DDChassis001System::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  // ---------- 串口与协议 ----------
  cfg_.serial_device = param(info, "serial_device", cfg_.serial_device);
  cfg_.baud_rate = param_int(info, "baud_rate", cfg_.baud_rate);
  cfg_.frame_end = param_u32(info, "frame_end", cfg_.frame_end);
  cfg_.frame_timeout_ms = param_int(info, "frame_timeout_ms", cfg_.frame_timeout_ms);
  cfg_.timeout_stops_motors = param_bool(info, "timeout_stops_motors", cfg_.timeout_stops_motors);
  cfg_.max_wheel_velocity = param_double(info, "max_wheel_velocity", cfg_.max_wheel_velocity);
  cfg_.command_epsilon = param_double(info, "command_epsilon", cfg_.command_epsilon);
  if (cfg_.frame_end == 0) {
    RCLCPP_ERROR(logger(), "必须配置 frame_end（固件帧尾魔数），否则无法对齐数据帧");
    return CallbackReturn::ERROR;
  }

  // ---------- 两轮映射 ----------
  WheelConfig left;
  left.joint_name = param(info, "left_wheel_joint", "wheel_l");
  left.motor_index = param_int(info, "left_motor_index", 0);
  left.speed_scale = param_double(info, "left_speed_scale", 1.0);
  left.invert = param_bool(info, "left_invert", false);

  WheelConfig right;
  right.joint_name = param(info, "right_wheel_joint", "wheel_r");
  right.motor_index = param_int(info, "right_motor_index", 1);
  right.speed_scale = param_double(info, "right_speed_scale", 1.0);
  right.invert = param_bool(info, "right_invert", false);

  wheels_ = {left, right};
  for (const auto & wheel : wheels_) {
    const bool found = std::any_of(
      info.joints.begin(), info.joints.end(),
      [&wheel](const auto & joint) {return joint.name == wheel.joint_name;});
    if (!found) {
      RCLCPP_ERROR(logger(), "URDF 里找不到关节 \"%s\"", wheel.joint_name.c_str());
      return CallbackReturn::ERROR;
    }
  }

  const size_t wheel_count = wheels_.size();
  hw_position_.assign(wheel_count, 0.0);
  hw_velocity_.assign(wheel_count, 0.0);
  hw_velocity_cmd_.assign(wheel_count, 0.0);
  last_sent_cmd_.assign(wheel_count, 0.0);
  has_sent_cmd_.assign(wheel_count, false);

  // ---------- IMU ----------
  cfg_.publish_imu = param_bool(info, "publish_imu", cfg_.publish_imu);
  cfg_.imu_topic = param(info, "imu_topic", cfg_.imu_topic);
  cfg_.imu_frame_id = param(info, "imu_frame_id", cfg_.imu_frame_id);
  cfg_.imu_yaw_field = param_int(info, "imu_yaw_field", cfg_.imu_yaw_field);
  cfg_.imu_angle_is_deg = param_bool(info, "imu_angle_is_deg", cfg_.imu_angle_is_deg);
  cfg_.imu_yaw_sign = param_double(info, "imu_yaw_sign", cfg_.imu_yaw_sign);
  cfg_.imu_publish_rate = param_double(info, "imu_publish_rate", cfg_.imu_publish_rate);
  if (cfg_.imu_yaw_field < 1 || cfg_.imu_yaw_field > 3) {
    RCLCPP_ERROR(logger(), "imu_yaw_field 必须是 1~3（分别对应 imu1/imu2/imu3），当前 %d",
      cfg_.imu_yaw_field);
    return CallbackReturn::ERROR;
  }

  // ---------- 打开串口 ----------
  protocol_ = ChassisProtocol(cfg_.frame_end);
  std::string error;
  if (!port_.open(cfg_.serial_device, cfg_.baud_rate, error)) {
    RCLCPP_ERROR(logger(), "%s", error.c_str());
    return CallbackReturn::ERROR;
  }
  RCLCPP_INFO(logger(), "底盘串口已打开: %s @ %d, frame_end=0x%08X",
    cfg_.serial_device.c_str(), cfg_.baud_rate, cfg_.frame_end);

  // 固件上电默认目标速度未知，先显式置零
  for (size_t i = 0; i < wheel_count; ++i) {
    send_wheel_command(i, 0.0);
  }
  last_frame_time_ = SteadyClock::now();

  // ---------- IMU 发布线程（Humble 没有 imu_sensor_broadcaster，只能组件内自建节点发布） ----------
  if (cfg_.publish_imu) {
    imu_node_ = std::make_shared<rclcpp::Node>("dd_chassis_001_imu");
    imu_node_->set_parameter(rclcpp::Parameter("use_sim_time", false));
    imu_pub_ = imu_node_->create_publisher<sensor_msgs::msg::Imu>(
      cfg_.imu_topic, rclcpp::QoS(10));
    imu_thread_running_ = true;
    imu_thread_ = std::thread(&DDChassis001System::imu_publish_loop, this);
    RCLCPP_INFO(logger(), "IMU 发布已启动: topic=%s frame=%s rate=%.0fHz",
      cfg_.imu_topic.c_str(), cfg_.imu_frame_id.c_str(), cfg_.imu_publish_rate);
  }

  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DDChassis001System::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  for (size_t i = 0; i < wheels_.size(); ++i) {
    send_wheel_command(i, 0.0);
  }
  has_sent_cmd_.assign(wheels_.size(), false);
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> DDChassis001System::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> interfaces;
  for (size_t i = 0; i < wheels_.size(); ++i) {
    // diff_drive_controller 用 position 差分算速度，velocity 作为直接反馈
    interfaces.emplace_back(wheels_[i].joint_name, "position", &hw_position_[i]);
    interfaces.emplace_back(wheels_[i].joint_name, "velocity", &hw_velocity_[i]);
  }
  return interfaces;
}

std::vector<hardware_interface::CommandInterface> DDChassis001System::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> interfaces;
  for (size_t i = 0; i < wheels_.size(); ++i) {
    interfaces.emplace_back(wheels_[i].joint_name, "velocity", &hw_velocity_cmd_[i]);
  }
  return interfaces;
}

hardware_interface::return_type DDChassis001System::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  uint8_t buffer[256];
  ChassisFrame frame{};

  // 把串口里积压的字节全部读掉，一次可能有 0~n 帧
  for (;;) {
    const ssize_t n = port_.read(buffer, sizeof(buffer));
    if (n < 0) {
      RCLCPP_ERROR_THROTTLE(logger(), throttle_clock(), 1000,
        "串口读失败，请检查 %s 是否被拔出", cfg_.serial_device.c_str());
      break;
    }
    if (n == 0) {
      break;
    }
    if (protocol_.push(buffer, static_cast<size_t>(n), frame)) {
      update_from_frame(frame);
    }
  }

  const double dt = period.seconds();
  const auto timeout = std::chrono::milliseconds(cfg_.frame_timeout_ms);
  comm_alive_ = got_frame_ && (SteadyClock::now() - last_frame_time_ <= timeout);

  if (comm_alive_) {
    for (size_t i = 0; i < wheels_.size(); ++i) {
      hw_position_[i] += hw_velocity_[i] * dt;   // 位置由速度积分得到（固件只报速度）
    }
  } else {
    // 通信中断：上报零速度；停机由 write() 按 timeout_stops_motors 处理
    for (size_t i = 0; i < wheels_.size(); ++i) {
      hw_velocity_[i] = 0.0;
    }
    RCLCPP_WARN_THROTTLE(logger(), throttle_clock(), 1000,
      "超过 %d ms 未收到有效数据帧，底盘通信可能中断", cfg_.frame_timeout_ms);
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type DDChassis001System::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  for (size_t i = 0; i < wheels_.size(); ++i) {
    double target = hw_velocity_cmd_[i];
    // 通信中断时强制停机，避免底盘按最后一次指令一直跑
    if (!comm_alive_ && cfg_.timeout_stops_motors) {
      target = 0.0;
    }
    if (cfg_.max_wheel_velocity > 0.0) {
      target = std::clamp(target, -cfg_.max_wheel_velocity, cfg_.max_wheel_velocity);
    }

    // 指令几乎没变就不发，减少串口占用（固件会保持上一次的目标值）
    if (has_sent_cmd_[i] && std::fabs(target - last_sent_cmd_[i]) < cfg_.command_epsilon) {
      continue;
    }
    send_wheel_command(i, target);
  }
  return hardware_interface::return_type::OK;
}

void DDChassis001System::send_wheel_command(size_t wheel, double joint_velocity)
{
  if (wheel >= wheels_.size() || !port_.is_open()) {
    return;
  }

  const WheelConfig & cfg = wheels_[wheel];
  if (cfg.speed_scale == 0.0) {
    RCLCPP_ERROR(logger(), "关节 %s 的 speed_scale 不能为 0", cfg.joint_name.c_str());
    return;
  }

  // 固件值 = 关节角速度 / 换算系数，方向相反时取反
  double firmware_value = joint_velocity / cfg.speed_scale;
  if (cfg.invert) {
    firmware_value = -firmware_value;
  }

  if (!port_.write(ChassisProtocol::speed_command(cfg.motor_index, firmware_value))) {
    RCLCPP_ERROR_THROTTLE(logger(), throttle_clock(), 1000,
      "速度指令发送失败");
    return;
  }
  last_sent_cmd_[wheel] = joint_velocity;
  has_sent_cmd_[wheel] = true;
}

void DDChassis001System::update_from_frame(const ChassisFrame & frame)
{
  const auto now = SteadyClock::now();
  last_frame_time_ = now;
  got_frame_ = true;

  for (size_t i = 0; i < wheels_.size(); ++i) {
    const size_t field = (wheels_[i].motor_index == 0) ? kFieldSpeed0 : kFieldSpeed1;
    const double direction = wheels_[i].invert ? -1.0 : 1.0;
    hw_velocity_[i] = frame[field] * wheels_[i].speed_scale * direction;
  }

  // ---------- IMU ----------
  const double angle_scale = cfg_.imu_angle_is_deg ? kPi / 180.0 : 1.0;
  const double raw_yaw = frame[kFieldImu1 + static_cast<size_t>(cfg_.imu_yaw_field - 1)];
  const double yaw = wrap_angle(raw_yaw * angle_scale * cfg_.imu_yaw_sign);

  std::lock_guard<std::mutex> lock(imu_mutex_);
  if (imu_valid_) {
    const double dt = std::chrono::duration<double>(now - last_imu_time_).count();
    if (dt > 1e-4) {
      // 固件只给欧拉角，角速度靠相邻帧差分得到（处理 ±180° 跳变）
      imu_yaw_rate_ = wrap_angle(yaw - imu_yaw_rad_) / dt;
    }
  }
  imu_yaw_rad_ = yaw;
  imu_valid_ = true;
  last_imu_time_ = now;
}

void DDChassis001System::imu_publish_loop()
{
  const auto period = std::chrono::duration<double>(
    1.0 / std::max(1.0, cfg_.imu_publish_rate));
  const auto timeout = std::chrono::milliseconds(cfg_.frame_timeout_ms);

  while (imu_thread_running_) {
    sensor_msgs::msg::Imu msg;
    msg.header.stamp = imu_node_->now();
    msg.header.frame_id = cfg_.imu_frame_id;

    bool valid = false;
    {
      std::lock_guard<std::mutex> lock(imu_mutex_);
      // 数据过旧就不发，让下游 EKF 走 sensor_timeout 逻辑
      valid = imu_valid_ && (SteadyClock::now() - last_imu_time_ <= timeout);
      if (valid) {
        const double half = imu_yaw_rad_ * 0.5;
        msg.orientation.x = 0.0;
        msg.orientation.y = 0.0;
        msg.orientation.z = std::sin(half);
        msg.orientation.w = std::cos(half);
        msg.angular_velocity.z = imu_yaw_rate_;
      }
    }

    if (valid) {
      // 只有 yaw 可信：roll/pitch 与 x/y 角速度协方差置大值，让融合只采信 yaw
      msg.orientation_covariance[0] = 1.0e6;
      msg.orientation_covariance[4] = 1.0e6;
      msg.orientation_covariance[8] = 1.0e-3;
      msg.angular_velocity_covariance[0] = 1.0e6;
      msg.angular_velocity_covariance[4] = 1.0e6;
      msg.angular_velocity_covariance[8] = 1.0e-2;
      msg.linear_acceleration_covariance[0] = -1.0;   // 固件不提供加速度
      imu_pub_->publish(msg);
    }

    std::this_thread::sleep_for(period);
  }
}

}  // namespace dd_chassis_001

PLUGINLIB_EXPORT_CLASS(dd_chassis_001::DDChassis001System, hardware_interface::SystemInterface)
