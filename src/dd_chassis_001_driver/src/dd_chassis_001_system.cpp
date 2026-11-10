/**
 * @file dd_chassis_001_system.cpp
 * @brief DDChassis001System 实现。
 *
 * 线程与数据面约定见 note/设计说明.md §2：
 *   IO 线程独占串口 -> 快照 -> read()/IMU 线程；write() 只往命令槽填值。
 */
#include "dd_chassis_001_driver/dd_chassis_001_system.hpp"

#include <poll.h>
#include <sys/eventfd.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstring>
#include <limits>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace dd_chassis_001_driver
{
namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kTwoPi = 2.0 * kPi;

/// 统一的日志名
rclcpp::Logger logger()
{
  return rclcpp::get_logger("dd_chassis_001");
}

/**
 * @brief 把角度归一化到 (-pi, pi]。
 */
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

rclcpp::Clock & DDChassis001System::throttle_clock()
{
  static rclcpp::Clock clock(RCL_STEADY_TIME);
  return clock;
}

DDChassis001System::~DDChassis001System()
{
  // 兜底：即使生命周期没走完也要让线程退出
  io_running_ = false;
  imu_running_ = false;
  wake_io();
  frame_signal_.wake_all();
  if (io_thread_.joinable()) {
    io_thread_.join();
  }
  if (imu_thread_.joinable()) {
    imu_thread_.join();
  }
  if (event_fd_ >= 0) {
    ::close(event_fd_);
    event_fd_ = -1;
  }
  port_.close();
}

// ==========================================================================
// 生命周期
// ==========================================================================

hardware_interface::CallbackReturn DDChassis001System::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // ---------- 1) 参数解析与校验（纯逻辑）----------
  std::string error_message;
  const ParseResult result = parse_config(info.hardware_parameters, cfg_, error_message);
  if (result != ParseResult::kOk) {
    const char * kind = (result == ParseResult::kMissing) ? "参数缺失" : "参数非法";
    RCLCPP_ERROR(logger(), "%s: %s", kind, error_message.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  // ---------- 2) 交叉校验：配置的 joint 名必须真的存在于 URDF ----------
  bool left_found = false;
  bool right_found = false;
  for (const auto & joint : info.joints) {
    left_found = left_found || (joint.name == cfg_.left.joint_name);
    right_found = right_found || (joint.name == cfg_.right.joint_name);
  }
  if (!left_found || !right_found) {
    RCLCPP_ERROR(
      logger(), "URDF 中找不到关节: left=\"%s\"(%d) right=\"%s\"(%d)",
      cfg_.left.joint_name.c_str(), static_cast<int>(left_found),
      cfg_.right.joint_name.c_str(), static_cast<int>(right_found));
    return hardware_interface::CallbackReturn::ERROR;
  }

  protocol_ = ChassisProtocol(cfg_.frame_end);

  RCLCPP_INFO(
    logger(),
    "配置: %s @ %d  frame_end=0x%08X  timeout=%dms  "
    "left=%s(motor %d, scale %.6f, invert %d)  right=%s(motor %d, scale %.6f, invert %d)",
    cfg_.serial_device.c_str(), cfg_.baud_rate, cfg_.frame_end, cfg_.frame_timeout_ms,
    cfg_.left.joint_name.c_str(), cfg_.left.motor_index, cfg_.left.speed_scale,
    static_cast<int>(cfg_.left.invert),
    cfg_.right.joint_name.c_str(), cfg_.right.motor_index, cfg_.right.speed_scale,
    static_cast<int>(cfg_.right.invert));

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DDChassis001System::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  std::string error;
  if (!port_.open(cfg_.serial_device, cfg_.baud_rate, error)) {
    RCLCPP_ERROR(logger(), "%s", error.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  // eventfd 用于让 write() 立刻唤醒阻塞在 poll() 上的 IO 线程
  event_fd_ = ::eventfd(0, EFD_NONBLOCK | EFD_CLOEXEC);
  if (event_fd_ < 0) {
    RCLCPP_ERROR(logger(), "eventfd 创建失败: %s", std::strerror(errno));
    port_.close();
    return hardware_interface::CallbackReturn::ERROR;
  }

  protocol_.reset();
  first_frame_ = true;
  has_sent_ = false;
  last_sent_velocity_ = {0.0, 0.0};
  logged_alive_ = true;

  io_running_ = true;
  io_thread_ = std::thread(&DDChassis001System::io_loop, this);

  RCLCPP_INFO(logger(), "串口已打开并启动 IO 线程: %s", cfg_.serial_device.c_str());
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DDChassis001System::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  io_running_ = false;
  wake_io();
  if (io_thread_.joinable()) {
    io_thread_.join();
  }
  if (event_fd_ >= 0) {
    ::close(event_fd_);
    event_fd_ = -1;
  }
  port_.close();
  RCLCPP_INFO(logger(), "串口已关闭，IO 线程已退出");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DDChassis001System::on_shutdown(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // 停机属于安全例外：允许直接写一次，不受"回调不碰串口"约束
  send_stop();
  io_running_ = false;
  imu_running_ = false;
  wake_io();
  frame_signal_.wake_all();
  if (io_thread_.joinable()) {
    io_thread_.join();
  }
  if (imu_thread_.joinable()) {
    imu_thread_.join();
  }
  if (event_fd_ >= 0) {
    ::close(event_fd_);
    event_fd_ = -1;
  }
  port_.close();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DDChassis001System::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  wheel_position_[0] = wheel_position_[1] = 0.0;
  wheel_velocity_cmd_[0] = wheel_velocity_cmd_[1] = 0.0;

  if (!cfg_.publish_imu) {
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  imu_node_ = std::make_shared<rclcpp::Node>("dd_chassis_001_imu");
  imu_publisher_ = imu_node_->create_publisher<sensor_msgs::msg::Imu>(
    cfg_.imu_topic, rclcpp::QoS(10).best_effort());
  ros_time_offset_ = imu_node_->now() - rclcpp::Time(0, 0, RCL_ROS_TIME);
  ros_time_offset_stamp_ = SteadyClock::now();

  imu_running_ = true;
  imu_thread_ = std::thread(&DDChassis001System::imu_loop, this);

  RCLCPP_INFO(
    logger(), "IMU 发布已启动: topic=%s frame=%s rate<=%.1fHz",
    cfg_.imu_topic.c_str(), cfg_.imu_frame_id.c_str(), cfg_.imu_publish_rate);
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DDChassis001System::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // 只填命令槽（不碰串口），让 IO 线程立即可靠停机
  command_slot_.set({0.0, 0.0}, true);
  wake_io();
  last_sent_velocity_ = {0.0, 0.0};
  has_sent_ = false;

  imu_running_ = false;
  frame_signal_.wake_all();
  if (imu_thread_.joinable()) {
    imu_thread_.join();
  }
  imu_publisher_.reset();
  imu_node_.reset();

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DDChassis001System::on_error(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  send_stop();
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ==========================================================================
// 接口导出
// ==========================================================================

std::vector<hardware_interface::StateInterface> DDChassis001System::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  state_interfaces.emplace_back(
    cfg_.left.joint_name, hardware_interface::HW_IF_POSITION, &wheel_position_[0]);
  state_interfaces.emplace_back(
    cfg_.left.joint_name, hardware_interface::HW_IF_VELOCITY, &wheel_velocity_[0]);
  state_interfaces.emplace_back(
    cfg_.right.joint_name, hardware_interface::HW_IF_POSITION, &wheel_position_[1]);
  state_interfaces.emplace_back(
    cfg_.right.joint_name, hardware_interface::HW_IF_VELOCITY, &wheel_velocity_[1]);
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> DDChassis001System::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  command_interfaces.emplace_back(
    cfg_.left.joint_name, hardware_interface::HW_IF_VELOCITY, &wheel_velocity_cmd_[0]);
  command_interfaces.emplace_back(
    cfg_.right.joint_name, hardware_interface::HW_IF_VELOCITY, &wheel_velocity_cmd_[1]);
  return command_interfaces;
}

// ==========================================================================
// 控制周期回调（纯内存，无 syscall）
// ==========================================================================

hardware_interface::return_type DDChassis001System::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // IO 线程已经完成积分，这里只做整块拷贝
  const StateSnapshot snapshot = snapshot_.get();
  wheel_velocity_[0] = snapshot.wheel_velocity[0];
  wheel_velocity_[1] = snapshot.wheel_velocity[1];
  wheel_position_[0] = snapshot.wheel_position[0];
  wheel_position_[1] = snapshot.wheel_position[1];

  if (!snapshot.comm_alive) {
    // 通信中断：上报 0，但不返回 ERROR（避免控制器停机后遗留速度）
    wheel_velocity_[0] = 0.0;
    wheel_velocity_[1] = 0.0;
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type DDChassis001System::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  std::array<double, 2> target{};
  double * cmds[kWheelCount] = {&wheel_velocity_cmd_[0], &wheel_velocity_cmd_[1]};

  for (size_t i = 0; i < kWheelCount; ++i) {
    double value = *cmds[i];
    if (!std::isfinite(value)) {
      value = 0.0;
    }
    if (cfg_.max_wheel_velocity > 0.0) {
      value = std::clamp(value, -cfg_.max_wheel_velocity, cfg_.max_wheel_velocity);
    }
    target[i] = value;
  }

  // 变化量小于 epsilon 就不打扰 IO 线程（固件会保持上次目标值）
  const bool changed =
    std::fabs(target[0] - last_sent_velocity_[0]) >= cfg_.command_epsilon ||
    std::fabs(target[1] - last_sent_velocity_[1]) >= cfg_.command_epsilon;

  if (changed || !has_sent_) {
    command_slot_.set(target, false);
    wake_io();
  }
  return hardware_interface::return_type::OK;
}

// ==========================================================================
// IO 线程
// ==========================================================================

void DDChassis001System::wake_io()
{
  if (event_fd_ >= 0) {
    const uint64_t one = 1;
    // 写 eventfd 可能因计数溢出返回 EAGAIN，忽略即可
    const ssize_t n = ::write(event_fd_, &one, sizeof(one));
    (void)n;
  }
}

void DDChassis001System::send_velocity(const std::array<double, 2> & velocity)
{
  const WheelConfig * wheels[kWheelCount] = {&cfg_.left, &cfg_.right};
  for (size_t i = 0; i < kWheelCount; ++i) {
    const WheelConfig & wheel = *wheels[i];
    if (wheel.speed_scale <= 0.0) {
      continue;
    }
    // 下发是"除"：固件值 = 关节角速度 / 换算系数
    double firmware_value = velocity[i] / wheel.speed_scale;
    if (wheel.invert) {
      firmware_value = -firmware_value;
    }
    if (!port_.write(ChassisProtocol::speed_command(wheel.motor_index, firmware_value))) {
      RCLCPP_ERROR_THROTTLE(logger(), throttle_clock(), 1000, "速度指令发送失败");
    }
  }

  last_sent_velocity_ = velocity;
  has_sent_ = true;
  last_send_time_ = SteadyClock::now();
}

void DDChassis001System::send_stop()
{
  if (!port_.is_open()) {
    return;
  }
  const WheelConfig * wheels[kWheelCount] = {&cfg_.left, &cfg_.right};
  for (size_t i = 0; i < kWheelCount; ++i) {
    port_.write(ChassisProtocol::speed_command(wheels[i]->motor_index, 0.0));
  }
  last_sent_velocity_ = {0.0, 0.0};
  has_sent_ = true;
}

bool DDChassis001System::reopen_serial()
{
  port_.close();

  std::string error;
  if (!port_.open(cfg_.serial_device, cfg_.baud_rate, error)) {
    RCLCPP_ERROR_THROTTLE(
      logger(), throttle_clock(), 2000, "重新打开串口失败: %s", error.c_str());
    return false;
  }

  // 旧 fd 里可能还有半帧数据，清空解析缓冲避免拼出错误帧
  protocol_.reset();

  // 重新起算：第一帧不参与积分，避免断连期间的时间差被算成位移
  first_frame_ = true;
  last_frame_time_ = SteadyClock::now();
  has_sent_ = false;
  logged_alive_ = false;   // 下一帧到达时会打印"底盘通信恢复"

  return true;
}

void DDChassis001System::apply_frame(const ChassisFrame & frame, SteadyClock::time_point now)
{
  const WheelConfig * wheels[kWheelCount] = {&cfg_.left, &cfg_.right};
  const size_t speed_fields[kWheelCount] = {kFieldSpeed0, kFieldSpeed1};

  StateSnapshot snapshot = snapshot_.get();
  if (!snapshot.valid) {
    snapshot = StateSnapshot{};  // 第一帧：从零开始积分
  }

  // 位置积分用真实帧间隔（比控制周期更准）
  double dt = 0.0;
  if (snapshot.valid) {
    dt = std::chrono::duration<double>(now - snapshot.stamp).count();
    // 串口断续时会出现超大间隔（断连 3s 后恢复）。
    // 这种间隔不能用来积分，否则 wheel_position 会一下子跳 v*3s 的距离。
    constexpr double kMaxFrameGap = 0.5;  // [s]
    if (dt > kMaxFrameGap) {
      dt = 0.0;
      snapshot.imu_yaw_rate = 0.0;
    }
  }

  for (size_t i = 0; i < kWheelCount; ++i) {
    const WheelConfig & wheel = *wheels[i];
    const double raw = frame[speed_fields[i]];
    const double direction = wheel.invert ? -1.0 : 1.0;
    snapshot.wheel_velocity[i] = raw * wheel.speed_scale * direction;
    snapshot.wheel_position[i] += snapshot.wheel_velocity[i] * dt;
  }

  // ---- IMU：只取 yaw，角速度由相邻帧差分 ----
  const double angle_scale = cfg_.imu_angle_is_deg ? (kPi / 180.0) : 1.0;
  const size_t yaw_field = kFieldImu1 + static_cast<size_t>(cfg_.imu_yaw_field - 1);
  double yaw = wrap_angle(frame[yaw_field] * angle_scale * cfg_.imu_yaw_sign);

  // 差分算角速度有两个坑：
  //   1) dt 过小：串口缓冲区积压后会一次性吐出多帧，相邻帧时间戳几乎相同，
  //      Δyaw/dt 会算出几百上千 rad/s。EKF 拿它积分后 yaw 剧烈乱转
  //      （实测 1 秒跳 215°）。
  //   2) 真的大角速度也不应在物理范围外。
  // 所以既限 dt 下限，也对结果做钳位。
  constexpr double kMinDt = 0.002;       // [s] 对应 500Hz，高于上报率 120Hz 的两倍
  constexpr double kMaxYawRate = 10.0;   // [rad/s] 物理合理上限
  if (snapshot.valid && dt > kMinDt) {
    const double rate = wrap_angle(yaw - snapshot.imu_yaw_rad) / dt;
    snapshot.imu_yaw_rate = std::clamp(rate, -kMaxYawRate, kMaxYawRate);
  } else {
    // dt 缺失或过小：不给角速度，宁可这一帧不给，也不要给错的
    snapshot.imu_yaw_rate = 0.0;
  }
  snapshot.imu_yaw_rad = yaw;
  snapshot.stamp = now;
  snapshot.valid = true;
  snapshot.comm_alive = true;
  snapshot_.publish(snapshot);

  if (cfg_.debug_log) {
    RCLCPP_INFO_THROTTLE(
      logger(), throttle_clock(), 1000,
      "[dbg] speed raw=[%.3f, %.3f] -> vel=[%.3f, %.3f] pos=[%.3f, %.3f] yaw=%+.1fdeg",
      frame[kFieldSpeed0], frame[kFieldSpeed1],
      snapshot.wheel_velocity[0], snapshot.wheel_velocity[1],
      snapshot.wheel_position[0], snapshot.wheel_position[1],
      snapshot.imu_yaw_rad * 180.0 / kPi);
  }

  frame_signal_.notify();
}

void DDChassis001System::io_loop()
{
  uint8_t buffer[512];
  ChassisFrame frame{};
  const auto timeout = std::chrono::milliseconds(cfg_.frame_timeout_ms);
  // poll 上限 100ms：既保证命令及时响应，也保证停止标志能被看到
  constexpr int kPollTimeoutMs = 100;

  while (io_running_) {
    struct pollfd fds[2];
    fds[0].fd = port_.fd();
    fds[0].events = POLLIN;
    fds[1].fd = event_fd_;
    fds[1].events = POLLIN;

    const int ready = ::poll(fds, 2, kPollTimeoutMs);
    if (ready < 0 && errno != EINTR) {
      RCLCPP_ERROR_THROTTLE(logger(), throttle_clock(), 1000, "poll 失败: %s", std::strerror(errno));
    }

    // ---------- 1) 读串口：一次读干净，可能含多帧 ----------
    if (ready > 0) {
      // 串口断开/出错：poll 会返回 POLLHUP/POLLERR/POLLNVAL。
      // 原来只处理 POLLIN，导致 USB 拔插后旧 fd 失效但仍在空转，
      // 不重开就永远收不到数据，也永远不会打印“通信恢复”。
      if (fds[0].revents & (POLLHUP | POLLERR | POLLNVAL)) {
        RCLCPP_ERROR_THROTTLE(
          logger(), throttle_clock(), 2000,
          "串口异常 (revents=0x%x)，尝试重新打开 %s",
          fds[0].revents, cfg_.serial_device.c_str());
        if (reopen_serial()) {
          RCLCPP_INFO(logger(), "串口已重新打开: %s", cfg_.serial_device.c_str());
        } else {
          // 设备可能还没插回来，等一会儿再试，避免刷爆日志
          std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
        continue;
      }

      if (fds[0].revents & POLLIN) {
        for (;; ) {
          const ssize_t n = port_.read(buffer, sizeof(buffer));
          if (n <= 0) {
            break;
          }
          if (protocol_.push(buffer, static_cast<size_t>(n), frame)) {
            apply_frame(frame, SteadyClock::now());
            last_frame_time_ = SteadyClock::now();
            first_frame_ = false;
            if (!logged_alive_) {
              RCLCPP_INFO(logger(), "底盘通信恢复");
              logged_alive_ = true;
            }
          }
        }
      }
    }

    // ---------- 2) 清空 eventfd 计数 ----------
    if (ready > 0 && (fds[1].revents & POLLIN)) {
      uint64_t value = 0;
      const ssize_t n = ::read(event_fd_, &value, sizeof(value));
      (void)n;
    }

    // ---------- 3) 下发命令（优先处理安全命令）----------
    std::array<double, 2> velocity{};
    if (command_slot_.take(velocity)) {
      send_velocity(velocity);
    } else if (cfg_.command_repeat_period_ms > 0 && has_sent_ &&
      (SteadyClock::now() - last_send_time_) >=
      std::chrono::milliseconds(cfg_.command_repeat_period_ms))
    {
      send_velocity(last_sent_velocity_);
    }

    // ---------- 4) 超时判定 ----------
    const bool alive = !first_frame_ && (SteadyClock::now() - last_frame_time_ <= timeout);
    if (!alive) {
      StateSnapshot snapshot = snapshot_.get();
      if (snapshot.comm_alive) {
        // 刚判定断线：上报零速度并（按配置）让固件停机
        snapshot.wheel_velocity = {0.0, 0.0};
        snapshot.imu_yaw_rate = 0.0;
        snapshot.comm_alive = false;
        snapshot_.publish(snapshot);
        if (cfg_.timeout_stops_motors) {
          send_stop();
        }
        RCLCPP_WARN_THROTTLE(
          logger(), throttle_clock(), 1000,
          "超过 %d ms 未收到有效数据帧，底盘通信中断", cfg_.frame_timeout_ms);
        logged_alive_ = false;
      }
    }
  }
}

// ==========================================================================
// IMU 线程
// ==========================================================================

rclcpp::Time DDChassis001System::to_ros_time(SteadyClock::time_point tp) const
{
  return rclcpp::Time(0, 0, RCL_ROS_TIME) + ros_time_offset_ +
         rclcpp::Duration(
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      tp - ros_time_offset_stamp_));
}

void DDChassis001System::imu_loop()
{
  // 抽帧间隔：上报 120Hz、配置 60Hz 时每两帧发一次
  const double period_s = 1.0 / std::max(1e-3, cfg_.imu_publish_rate);
  uint64_t last_sequence = 0;
  SteadyClock::time_point last_publish{};

  while (imu_running_) {
    if (!frame_signal_.wait_for_new(last_sequence, std::chrono::milliseconds(200))) {
      continue;  // 超时/唤醒但无新帧
    }

    const StateSnapshot snapshot = snapshot_.get();
    if (!snapshot.valid || !snapshot.comm_alive) {
      continue;
    }

    // 抽帧：未到最小间隔就跳过（保证不重复发旧数据，也限制发布频率）
    if (last_publish.time_since_epoch().count() != 0 &&
      std::chrono::duration<double>(snapshot.stamp - last_publish).count() < period_s * 0.9)
    {
      continue;
    }
    last_publish = snapshot.stamp;

    sensor_msgs::msg::Imu msg;
    msg.header.stamp = to_ros_time(snapshot.stamp);
    msg.header.frame_id = cfg_.imu_frame_id;

    const double half = snapshot.imu_yaw_rad * 0.5;
    msg.orientation.x = 0.0;
    msg.orientation.y = 0.0;
    msg.orientation.z = std::sin(half);
    msg.orientation.w = std::cos(half);
    msg.angular_velocity.z = snapshot.imu_yaw_rate;

    // 只有 yaw 与 yaw 角速度可信，其余置大协方差；加速度不提供
    msg.orientation_covariance[0] = 1.0e6;
    msg.orientation_covariance[4] = 1.0e6;
    msg.orientation_covariance[8] = 1.0e-3;
    msg.angular_velocity_covariance[0] = 1.0e6;
    msg.angular_velocity_covariance[4] = 1.0e6;
    msg.angular_velocity_covariance[8] = 1.0e-2;
    msg.linear_acceleration_covariance[0] = -1.0;

    if (imu_publisher_) {
      imu_publisher_->publish(msg);
    }

    // 周期性校正 steady -> ROS 时间偏移（避免 now() 进入每帧热路径）
    if (cfg_.imu_stamp_offset_calib_s > 0.0 &&
      std::chrono::duration<double>(SteadyClock::now() - ros_time_offset_stamp_).count() >=
      cfg_.imu_stamp_offset_calib_s)
    {
      if (imu_node_) {
        ros_time_offset_ = imu_node_->now() - rclcpp::Time(0, 0, RCL_ROS_TIME);
        ros_time_offset_stamp_ = SteadyClock::now();
      }
    }
  }
}

}  // namespace dd_chassis_001_driver

// 导出给 pluginlib：第一个参数必须与 xml 中 <class type="..."> 一致
PLUGINLIB_EXPORT_CLASS(
  dd_chassis_001_driver::DDChassis001System, hardware_interface::SystemInterface)
