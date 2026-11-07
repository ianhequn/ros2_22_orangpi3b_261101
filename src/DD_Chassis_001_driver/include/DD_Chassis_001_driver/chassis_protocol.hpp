#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace dd_chassis_001
{
/// 固件上报帧：15 个 float32 + 1 个 uint32 帧尾 = 64 字节，低字节在前(小端)
constexpr size_t kFrameFloatCount = 15;
constexpr size_t kFrameSize = kFrameFloatCount * sizeof(float) + sizeof(uint32_t);

/// 帧内 float 字段下标，顺序与固件 README 的表格一致
enum FieldIndex : size_t
{
  kFieldImu1 = 0,
  kFieldImu2 = 1,
  kFieldImu3 = 2,
  kFieldError0 = 3,
  kFieldOutput0 = 4,
  kFieldError1 = 5,
  kFieldOutput1 = 6,
  kFieldKp0 = 7,
  kFieldKi0 = 8,
  kFieldKd0 = 9,
  kFieldKp1 = 10,
  kFieldKi1 = 11,
  kFieldKd1 = 12,
  kFieldSpeed0 = 13,
  kFieldSpeed1 = 14,
};

using ChassisFrame = std::array<float, kFrameFloatCount>;

/// 帧解析 + 命令组包。协议本身没有帧头，只能靠“帧尾魔数”在字节流里对齐。
class ChassisProtocol
{
public:
  explicit ChassisProtocol(uint32_t frame_end)
  : frame_end_(frame_end) {}

  /// 追加串口字节并尝试解析；解析到完整帧时写入 out 并返回 true
  bool push(const uint8_t * data, size_t len, ChassisFrame & out);

  /// 组速度指令："sz qeiM <motor_index> T <value>\n"
  static std::string speed_command(int motor_index, double value);

private:
  bool try_parse(ChassisFrame & out);

  uint32_t frame_end_;
  std::vector<uint8_t> buf_;
};

}  // namespace dd_chassis_001
