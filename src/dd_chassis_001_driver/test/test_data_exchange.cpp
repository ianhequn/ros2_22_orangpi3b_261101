/**
 * @file test_data_exchange.cpp
 * @brief 三个内存数据面（快照、命令槽、新帧事件）的行为测试。
 */
#include <chrono>
#include <thread>

#include "gtest/gtest.h"

#include "dd_chassis_001_driver/data_exchange.hpp"

using dd_chassis_001_driver::CommandSlot;
using dd_chassis_001_driver::FrameSignal;
using dd_chassis_001_driver::SnapshotBuffer;
using dd_chassis_001_driver::StateSnapshot;

TEST(TestSnapshotBuffer, publish_then_get_returns_latest)
{
  SnapshotBuffer buffer;
  StateSnapshot first;
  first.wheel_velocity = {1.0, 2.0};
  first.valid = true;
  buffer.publish(first);

  EXPECT_DOUBLE_EQ(buffer.get().wheel_velocity[0], 1.0);

  StateSnapshot second;
  second.wheel_velocity = {3.0, 4.0};
  second.valid = true;
  buffer.publish(second);

  const StateSnapshot got = buffer.get();
  EXPECT_DOUBLE_EQ(got.wheel_velocity[0], 3.0);
  EXPECT_DOUBLE_EQ(got.wheel_velocity[1], 4.0);
}

TEST(TestCommandSlot, empty_slot_returns_false)
{
  CommandSlot slot;
  std::array<double, 2> velocity{};
  EXPECT_FALSE(slot.take(velocity));
}

TEST(TestCommandSlot, take_clears_pending_and_overwrites)
{
  CommandSlot slot;
  std::array<double, 2> velocity{};

  slot.set({1.0, 2.0}, false);
  slot.set({3.0, 4.0}, false);  // 覆盖式：只保留最新
  ASSERT_TRUE(slot.take(velocity));
  EXPECT_DOUBLE_EQ(velocity[0], 3.0);
  EXPECT_DOUBLE_EQ(velocity[1], 4.0);

  EXPECT_FALSE(slot.take(velocity));  // 取走后为空
}

TEST(TestCommandSlot, priority_flag_is_exposed)
{
  CommandSlot slot;
  slot.set({0.0, 0.0}, true);
  EXPECT_TRUE(slot.priority_pending());

  std::array<double, 2> velocity{};
  ASSERT_TRUE(slot.take(velocity));
  EXPECT_FALSE(slot.priority_pending());
}

TEST(TestFrameSignal, wait_returns_false_on_timeout)
{
  FrameSignal signal;
  uint64_t seq = 0;
  EXPECT_FALSE(signal.wait_for_new(seq, std::chrono::milliseconds(20)));
}

TEST(TestFrameSignal, wait_returns_true_after_notify)
{
  FrameSignal signal;
  uint64_t seq = 0;

  std::thread notifier([&signal] {
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
      signal.notify();
    });

  const bool changed = signal.wait_for_new(seq, std::chrono::milliseconds(500));
  notifier.join();

  EXPECT_TRUE(changed);
  EXPECT_EQ(seq, 1u);
}
