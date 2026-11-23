/**
 * @file chassis_protocol.hpp
 * @brief 固件串口协议：64 字节上报帧的解析 + 速度命令的组包。
 *
 * 帧格式（固件 README）：15 个 float32 + 1 个 uint32 帧尾，共 64 字节，小端。
 * 协议没有帧头，只能靠帧尾魔数在字节流里对齐。
 * 纯 C++，不依赖 ROS，可直接单测。
 */
#ifndef DD_CHASSIS_001_DRIVER__CHASSIS_PROTOCOL_HPP_
#define DD_CHASSIS_001_DRIVER__CHASSIS_PROTOCOL_HPP_

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace dd_chassis_001_driver
{

/// 帧内 float 字段个数
constexpr size_t kFrameFloatCount = 15;
/// 完整帧字节数：15*4 + 4
constexpr size_t kFrameSize = kFrameFloatCount * sizeof(float) + sizeof(uint32_t);

/**
 * @brief 帧内 float 字段下标，顺序与固件 README 的表格一致。
 */
enum FieldIndex : size_t
{
  kFieldImu1 = 0,     ///< imu 欧拉角分量 1
  kFieldImu2 = 1,     ///< imu 欧拉角分量 2
  kFieldImu3 = 2,     ///< imu 欧拉角分量 3
  kFieldError0 = 3,   ///< 电机 0 速度误差
  kFieldOutput0 = 4,  ///< 电机 0 pid 输出
  kFieldError1 = 5,   ///< 电机 1 速度误差
  kFieldOutput1 = 6,  ///< 电机 1 pid 输出
  kFieldKp0 = 7,      ///< 电机 0 Kp
  kFieldKi0 = 8,      ///< 电机 0 Ki
  kFieldKd0 = 9,      ///< 电机 0 Kd
  kFieldKp1 = 10,     ///< 电机 1 Kp
  kFieldKi1 = 11,     ///< 电机 1 Ki
  kFieldKd1 = 12,     ///< 电机 1 Kd
  kFieldSpeed0 = 13,  ///< 电机 0 速度
  kFieldSpeed1 = 14,  ///< 电机 1 速度
};

/// 解析出的一帧数据
using ChassisFrame = std::array<float, kFrameFloatCount>;

/**
 * @brief 字节流式帧解析器 + 命令组包。
 *
 * 用法：反复 push() 串口读到的字节，返回 true 时 out 是一帧完整数据。
 */
class ChassisProtocol
{
public:
  /**
 * @param frame_end 帧尾魔数（来自配置）
 */
  explicit ChassisProtocol(uint32_t frame_end)
  : frame_end_(frame_end) {}

  /// @brief 重置内部缓冲（重新打开串口时调用，避免残留半帧）。
  void reset() {buf_.clear();}

  /**
 * @brief 追加串口字节并尝试解析。
 * @param[in] data 新到的字节
 * @param[in] len 字节数
 * @param[out] out 解析到完整帧时写入
 * @return 至少解析出一帧则为 true（一次 push 可能含多帧，取最后解析出的那帧）
 * @note 内部会循环解析直到没有完整帧，因此 out 是"本次最后解出的帧"。
 */
  bool push(const uint8_t * data, size_t len, ChassisFrame & out);

  /**
 * @brief 组速度指令。
 * @param motor_index 固件中的电机序号
 * @param value 固件速度值（已换算）
 * @return "SZ qeiM <i> T <value>\n"。前缀 SZ 必须大写，否则固件静默丢弃。
 */
  static std::string speed_command(int motor_index, double value);

private:
  /// 尝试从缓冲中取出一个完整帧，成功返回 true 并更新 out
  bool try_parse(ChassisFrame & out);

  uint32_t frame_end_{0};
  std::vector<uint8_t> buf_;
};

}  // namespace dd_chassis_001_driver

#endif  // DD_CHASSIS_001_DRIVER__CHASSIS_PROTOCOL_HPP_
