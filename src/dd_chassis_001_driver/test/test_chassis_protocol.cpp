/**
 * @file test_chassis_protocol.cpp
 * @brief 帧解析与命令组包的单元测试。
 *
 * 这是最容易悄悄出错的层，四个必测场景：
 *  1. 完整帧逐字节喂入；
 *  2. 前置随机垃圾；
 *  3. 中间丢字节后恢复；
 *  4. 多帧连发一次 push。
 */
#include <cstring>
#include <numeric>
#include <vector>

#include "gtest/gtest.h"

#include "dd_chassis_001_driver/chassis_protocol.hpp"

namespace
{
using dd_chassis_001_driver::ChassisFrame;
using dd_chassis_001_driver::ChassisProtocol;
using dd_chassis_001_driver::kFrameSize;

constexpr uint32_t kEnd = 0x7F800000u;

/// 造一帧：第 i 个 float 字段 = base + i，帧尾写魔数
std::vector<uint8_t> make_frame(float base, uint32_t end = kEnd)
{
  std::vector<uint8_t> frame(kFrameSize, 0);
  for (size_t i = 0; i < dd_chassis_001_driver::kFrameFloatCount; ++i) {
    const float value = base + static_cast<float>(i);
    std::memcpy(&frame[i * sizeof(float)], &value, sizeof(value));
  }
  std::memcpy(&frame[kFrameSize - sizeof(uint32_t)], &end, sizeof(end));
  return frame;
}
}  // namespace

TEST(TestChassisProtocol, parses_frame_byte_by_byte)
{
  ChassisProtocol protocol(kEnd);
  ChassisFrame out{};
  const auto frame = make_frame(100.0F);

  bool got = false;
  for (uint8_t byte : frame) {
    got = protocol.push(&byte, 1, out) || got;
  }

  ASSERT_TRUE(got);
  EXPECT_FLOAT_EQ(out[dd_chassis_001_driver::kFieldSpeed0], 113.0F);
  EXPECT_FLOAT_EQ(out[dd_chassis_001_driver::kFieldSpeed1], 114.0F);
  EXPECT_FLOAT_EQ(out[dd_chassis_001_driver::kFieldImu1], 100.0F);
}

TEST(TestChassisProtocol, skips_leading_garbage)
{
  ChassisProtocol protocol(kEnd);
  ChassisFrame out{};
  const auto frame = make_frame(7.0F);

  const std::vector<uint8_t> junk{0x11, 0x22, 0x33, 0x44, 0x55, 0x66};
  std::vector<uint8_t> stream = junk;
  stream.insert(stream.end(), frame.begin(), frame.end());

  ASSERT_TRUE(protocol.push(stream.data(), stream.size(), out));
  EXPECT_FLOAT_EQ(out[dd_chassis_001_driver::kFieldSpeed0], 20.0F);
}

TEST(TestChassisProtocol, recovers_after_dropped_bytes)
{
  ChassisProtocol protocol(kEnd);
  ChassisFrame out{};

  const auto first = make_frame(100.0F);
  const auto broken = make_frame(200.0F);
  const auto third = make_frame(300.0F);

  std::vector<uint8_t> stream;
  stream.insert(stream.end(), first.begin(), first.end());
  stream.insert(stream.end(), broken.begin() + 5, broken.end());  // 第二帧头部丢 5 字节
  stream.insert(stream.end(), third.begin(), third.end());

  ASSERT_TRUE(protocol.push(stream.data(), stream.size(), out));
  // 最后一帧必须完好（若实现提前 return，这里会解成残帧或 0）
  EXPECT_FLOAT_EQ(out[dd_chassis_001_driver::kFieldSpeed0], 313.0F);
}

TEST(TestChassisProtocol, parses_multiple_frames_in_one_push)
{
  ChassisProtocol protocol(kEnd);
  ChassisFrame out{};

  const auto a = make_frame(10.0F);
  const auto b = make_frame(20.0F);
  std::vector<uint8_t> stream;
  stream.insert(stream.end(), a.begin(), a.end());
  stream.insert(stream.end(), b.begin(), b.end());

  ASSERT_TRUE(protocol.push(stream.data(), stream.size(), out));
  EXPECT_FLOAT_EQ(out[dd_chassis_001_driver::kFieldSpeed0], 33.0F);  // 20+13
}

TEST(TestChassisProtocol, waits_for_more_data_when_incomplete)
{
  ChassisProtocol protocol(kEnd);
  ChassisFrame out{};
  const auto frame = make_frame(1.0F);

  // 只喂半帧：不应解出，也不应丢弃
  ASSERT_FALSE(protocol.push(frame.data(), kFrameSize / 2, out));
  // 再喂剩下：应能解出
  ASSERT_TRUE(protocol.push(frame.data() + kFrameSize / 2, kFrameSize - kFrameSize / 2, out));
  EXPECT_FLOAT_EQ(out[dd_chassis_001_driver::kFieldSpeed0], 14.0F);
}

TEST(TestChassisProtocol, command_uses_uppercase_sz_and_newline)
{
  const std::string cmd = ChassisProtocol::speed_command(1, -20.5);
  EXPECT_EQ(cmd, "SZ qeiM 1 T -20.500\n");
  EXPECT_EQ(cmd.back(), '\n');
}
