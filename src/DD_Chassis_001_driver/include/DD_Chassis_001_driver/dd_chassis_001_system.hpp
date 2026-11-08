/**
 * @file dd_chassis_001_system.hpp
 * @brief DD_Chassis_001 两轮差速底盘（含板载 IMU）的 ros2_control 硬件组件声明。
 *
 * 说明：
 *  - 本类只承载「ros2_control 接口 + 硬件生命周期」，串口与帧协议建议另拆
 *    serial_port.* / chassis_protocol.*（纯粹 C++，可脱离 ROS 单测）。
 *  - 生命周期由 controller_manager 驱动，顺序：
 *      on_init -> on_configure -> on_activate -> (read/write)*
 *              -> on_deactivate -> on_cleanup -> on_shutdown
 */
#ifndef DD_CHASSIS_001_DRIVER__DD_CHASSIS_001_SYSTEM_HPP_
#define DD_CHASSIS_001_DRIVER__DD_CHASSIS_001_SYSTEM_HPP_

#include <string>
#include <vector>

#include "DD_Chassis_001_driver/visibility_control.hpp"

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace dd_chassis_001_driver
{

/**
 * @brief 两轮差速底盘系统组件。
 *
 * 对外接口（接口名为 "<joint>/<interface>"，joint 名必须与 URDF 一致）：
 *  | 接口                        | 方向    | 单位    | 含义          |
 *  | --------------------------- | ------- | ------- | ------------- |
 *  | wheel_l/velocity            | command | rad/s   | 左轮目标角速度 |
 *  | wheel_r/velocity            | command | rad/s   | 右轮目标角速度 |
 *  | wheel_l/position            | state   | rad     | 左轮累计转角   |
 *  | wheel_l/velocity            | state   | rad/s   | 左轮实测角速度 |
 *  | wheel_r/position            | state   | rad     | 右轮累计转角   |
 *  | wheel_r/velocity            | state   | rad/s   | 右轮实测角速度 |
 */
class DDChassis001System : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(DDChassis001System)

  // 默认构造：不申请任何资源。
  DDChassis001System() = default;

  // 析构：确保串口关闭、后台线程退出。
  ~DDChassis001System() override = default;

  /**
 * @brief 解析并校验 URDF 参数（不碰硬件）。
 * @param info 硬件描述；参数位于 info.hardware_parameters（key->字符串）。
 * @return SUCCESS 参数合法；ERROR 则组件加载失败、控制器不会启动。
 */
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  /**
 * @brief 建立通信（打开串口、清缓冲等）。
 * @param previous_state 进入本状态前的状态。
 * @return SUCCESS / ERROR
 */
  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  /**
 * @brief 释放通信资源，on_configure 的逆操作。
 * @return SUCCESS / ERROR
 */
  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  /**
 * @brief 进程退出前收尾（给固件下发 0 速度、关闭串口）。
 * @return SUCCESS / ERROR
 */
  hardware_interface::CallbackReturn on_shutdown(
    const rclcpp_lifecycle::State & previous_state) override;

  /**
 * @brief 启用硬件（电机使能、清故障、初始化积分量）。
 * @return SUCCESS / ERROR
 */
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  /**
 * @brief 停用硬件，务必在此下发 0 速度让执行器停转。
 * @return SUCCESS / ERROR
 */
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  /**
 * @brief 进入 error 状态时的兜底处理（至少停机）。
 * @return SUCCESS / ERROR
 */
  hardware_interface::CallbackReturn on_error(
    const rclcpp_lifecycle::State & previous_state) override;

  /**
 * @brief 导出状态接口，指针指向本类成员（controller_manager 直接读取）。
 * @return 接口列表。
 * @warning 导出后指针不能因容器重新分配而失效：成员数组请用固定大小，勿中途 resize。
 */
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  /**
 * @brief 导出命令接口，指针指向本类成员（控制器写入后由 write() 下发）。
 * @return 接口列表。
 */
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  /**
 * @brief 周期读取硬件状态到导出的状态变量。
 * @param time   控制管理器当前时间。
 * @param period 距上次调用的周期（用于积分/差分，比自行计时准）。
 * @return OK 正常；ERROR 会使控制器停机，可恢复故障建议「告警 + 上报 0」而非 ERROR。
 */
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  /**
 * @brief 周期把命令接口的值下发给硬件。
 * @param time   当前时间。
 * @param period 控制周期。
 * @return OK / ERROR
 */
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  // 若需支持多种控制模式（position/velocity/effort 切换）再实现下面两个（可选）：
  // bool prepare_command_mode_switch(const std::vector<std::string> & start_interfaces,
  //                                  const std::vector<std::string> & stop_interfaces) override;
  // bool perform_command_mode_switch(const std::vector<std::string> & start_interfaces,
  //                                  const std::vector<std::string> & stop_interfaces) override;

private:
  // 关节索引：0=左轮，1=右轮（与成员数组下标一致）。
  static constexpr size_t kWheelCount = 2;

  // ---- 配置项：在 on_init 中从硬件参数解析，默认值仅作兜底 ----
  std::string serial_device_{"/dev/ttyUSB1"};       ///< 串口设备文件
  int baud_rate_{115200};                           ///< 波特率
  std::string left_joint_{"wheel_l"};               ///< 左轮关节名
  std::string right_joint_{"wheel_r"};              ///< 右轮关节名
  // TODO: 帧尾魔数、速度换算系数、方向 invert、限幅、通信超时等参数

  // ---- 接口数据：export_* 导出其地址，必须固定大小 ----
  double wheel_position_[kWheelCount] = {0.0, 0.0};           ///< 累计转角 [rad]
  double wheel_velocity_[kWheelCount] = {0.0, 0.0};           ///< 实测角速度 [rad/s]
  double wheel_velocity_cmd_[kWheelCount] = {0.0, 0.0};       ///< 目标角速度 [rad/s]
};

} // namespace dd_chassis_001_driver

#endif // DD_CHASSIS_001_DRIVER__DD_CHASSIS_001_SYSTEM_HPP_
