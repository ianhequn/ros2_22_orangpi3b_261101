/**
 * @file config.hpp
 * @brief 硬件参数的结构体定义与解析（把 URDF <param> 变成强类型配置）。
 *
 * 放在独立头文件的原因：解析与校验逻辑可以脱离 ros2_control 单测，
 * 且后续 serial_port / chassis_protocol 只需依赖本结构，不必知道 URDF。
 */
#ifndef DD_CHASSIS_001_DRIVER__CONFIG_HPP_
#define DD_CHASSIS_001_DRIVER__CONFIG_HPP_

#include <cstdint>
#include <string>
#include <unordered_map>

namespace dd_chassis_001_driver
{

/**
 * @brief 单个轮子（关节）的配置。
 */
struct WheelConfig
{
  std::string joint_name;      ///< URDF 中的 joint 名，用于 export_* 拼接接口名
  int motor_index{0};          ///< 固件中的电机序号（0 或 1），用于 "SZ qeiM <i> T <v>"
  double speed_scale{1.0};     ///< 换算系数，必须 > 0：关节角速度 = 固件 speed 值 * speed_scale
  bool invert{false};          ///< 方向取反；只表达方向，不改变比例
};

/**
 * @brief 硬件组件的全部配置。
 *
 * 所有字段都有兜底默认值，但**必填项**（见 parse_callback_return 中的校验）缺失时
 * 会拒绝加载，避免"静默用错参数"。
 */
struct Config
{
  /// 左右轮默认电机号必须不同，否则未显式配置时会被交叉校验拦下
  Config() {right.motor_index = 1;}

  // ---- 串口 ----
  std::string serial_device{"/dev/ttyUSB0"};  ///< 串口设备文件
  int baud_rate{115200};                      ///< 波特率
  uint32_t frame_end{0};                      ///< 帧尾魔数（uint32），对齐 64 字节帧的唯一依据；必填且非 0
  int frame_timeout_ms{500};                  ///< 超过该时长无有效帧即判定通信中断
  bool timeout_stops_motors{true};            ///< 通信中断时是否向固件下发 0 速度

  // ---- 控制 ----
  double max_wheel_velocity{25.0};  ///< 关节角速度限幅 [rad/s]；<=0 表示不限幅
  double command_epsilon{0.01};     ///< 指令变化小于该值不下发 [rad/s]
  int command_repeat_period_ms{0};  ///< >0 时按该周期重发（固件需要心跳时用），0=仅变化时发

  // ---- 两轮 ----
  WheelConfig left;   ///< 左轮（motor_index 默认 0）
  WheelConfig right;  ///< 右轮（motor_index 默认 1，在 Config 构造函数中设置）

  // ---- IMU ----
  bool publish_imu{true};           ///< 是否发布 IMU 话题
  std::string imu_topic{"imu"};     ///< IMU 话题名（相对名会落在组件节点命名空间下）
  std::string imu_frame_id{"imu_link"};  ///< IMU 消息的 frame_id，必须能在 TF 树中找到
  int imu_yaw_field{1};             ///< 用 imu1/imu2/imu3 中的哪一路作为 yaw（取值 1~3）
  bool imu_angle_is_deg{true};      ///< 固件欧拉角单位：true=度，false=弧度
  double imu_yaw_sign{1.0};         ///< yaw 方向修正，取 1 或 -1
  double imu_publish_rate{100.0};   ///< IMU 发布频率上限 [Hz]（作抽帧用，不是定时器频率）
  double imu_stamp_offset_calib_s{10.0};  ///< steady_clock → ROS time 偏移的校正周期 [s]

  // ---- 调试 ----
  bool debug_log{false};  ///< true 时周期性打印帧原始值、换算值与实际下发内容
};

/**
 * @brief 参数解析结果，用于区分"缺失"与"非法"两类错误。
 */
enum class ParseResult
{
  kOk,           ///< 解析成功
  kMissing,      ///< 必填项缺失
  kInvalid,      ///< 值存在但不合法（非数字、越界、为 0 等）
};

/**
 * @brief 从硬件参数表解析出 Config，并进行完整性/合法性校验。
 *
 * @param params URDF 中 <ros2_control>/<hardware> 下的 <param name="..."> 键值表
 *               （即 hardware_interface::HardwareInfo::hardware_parameters）。
 * @param[out] cfg 解析结果；失败时内容不可用。
 * @param[out] error_message 失败原因（含参数名），用于打印日志。
 * @return ParseResult
 *
 * @note 本函数不抛异常、不依赖 ROS 运行时，可直接单测。
 */
ParseResult parse_config(
  const std::unordered_map<std::string, std::string> & params,
  Config & cfg, std::string & error_message);

}  // namespace dd_chassis_001_driver

#endif  // DD_CHASSIS_001_DRIVER__CONFIG_HPP_
