#include "DD_Chassis_001_driver/chassis_protocol.hpp"

#include <cstdio>
#include <cstring>

namespace dd_chassis_001
{
namespace
{
/// 从字节流里读一个小端 uint32（不依赖主机字节序）
uint32_t read_le_u32(const uint8_t * p)
{
  return static_cast<uint32_t>(p[0]) |
         (static_cast<uint32_t>(p[1]) << 8) |
         (static_cast<uint32_t>(p[2]) << 16) |
         (static_cast<uint32_t>(p[3]) << 24);
}

/// 从字节流里读一个小端 float32
float read_le_f32(const uint8_t * p)
{
  const uint32_t bits = read_le_u32(p);
  float value = 0.0F;
  std::memcpy(&value, &bits, sizeof(value));
  return value;
}
}  // namespace

bool ChassisProtocol::push(const uint8_t * data, size_t len, ChassisFrame & out)
{
  buf_.insert(buf_.end(), data, data + len);

  // 一次可能收到多帧（120Hz 上报 + 控制周期读取），循环全部解析掉
  bool got = false;
  while (try_parse(out)) {
    got = true;
  }
  return got;
}

bool ChassisProtocol::try_parse(ChassisFrame & out)
{
  // 帧没有帧头：在缓冲区里找帧尾魔数，魔数前 64 字节即一帧
  for (;;) {
    size_t magic_pos = 0;
    bool found = false;
    for (size_t i = 0; i + sizeof(uint32_t) <= buf_.size(); ++i) {
      if (read_le_u32(&buf_[i]) == frame_end_) {
        magic_pos = i;
        found = true;
        break;
      }
    }

    if (!found) {
      // 没找到魔数：保留末尾不足一帧的字节（帧体可能还在路上，不能丢）
      if (buf_.size() > kFrameSize - 1) {
        buf_.erase(
          buf_.begin(),
          buf_.begin() + static_cast<long>(buf_.size() - (kFrameSize - 1)));
      }
      return false;
    }

    const size_t frame_end_pos = magic_pos + sizeof(uint32_t);
    if (frame_end_pos < kFrameSize) {
      // 调数出现过早，说明之前的残帧没对齐，丢弃到魔数之后继续找
      buf_.erase(buf_.begin(), buf_.begin() + static_cast<long>(frame_end_pos));
      continue;
    }

    const uint8_t * frame = &buf_[frame_end_pos - kFrameSize];
    for (size_t f = 0; f < kFrameFloatCount; ++f) {
      out[f] = read_le_f32(frame + f * sizeof(float));
    }
    buf_.erase(buf_.begin(), buf_.begin() + static_cast<long>(frame_end_pos));
    return true;
  }
}

std::string ChassisProtocol::speed_command(int motor_index, double value)
{
  char text[64] = {0};
  std::snprintf(text, sizeof(text), "sz qeiM %d T %.3f\n", motor_index, value);
  return std::string(text);
}

}  // namespace dd_chassis_001
