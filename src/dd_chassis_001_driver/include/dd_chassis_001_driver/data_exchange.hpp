/**
 * @file data_exchange.hpp
 * @brief 线程间的三个内存数据面：状态快照、命令槽、新帧事件。
 *
 * 设计约束（见 note/设计说明.md §2）：
 *  - 串口只有 IO 线程碰，ROS 回调不做任何 syscall；
 *  - IO 线程与 IMU 线程、控制器回调之间只通过本文件的结构交换数据。
 */
#ifndef DD_CHASSIS_001_DRIVER__DATA_EXCHANGE_HPP_
#define DD_CHASSIS_001_DRIVER__DATA_EXCHANGE_HPP_

#include <array>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <mutex>

namespace dd_chassis_001_driver
{

/**
 * @brief IO 线程产出的最新状态快照。
 *
 * 字段全是 POD，双缓冲读取时整块拷贝即可，读者无需加锁。
 */
struct StateSnapshot
{
  std::array<double, 2> wheel_velocity{0.0, 0.0};   ///< 轮角速度 [rad/s]
  std::array<double, 2> wheel_position{0.0, 0.0};   ///< 累计转角 [rad]
  double imu_yaw_rad{0.0};                          ///< yaw [rad]，(-pi, pi]
  double imu_yaw_rate{0.0};                         ///< yaw 角速度 [rad/s]（相邻帧差分）
  std::chrono::steady_clock::time_point stamp{};    ///< 帧到达时刻（steady）
  bool valid{false};                                ///< 是否已成功解析过至少一帧
  bool comm_alive{false};                           ///< 当前通信是否正常（由 IO 线程按超时判定）
};

/**
 * @brief 状态快照的双缓冲发布器。
 *
 * 写入方（IO 线程）：publish(snapshot) —— 写"后备"槽后原子切换索引。
 * 读取方（read()/IMU 线程）：get() —— 读当前"前台"槽的拷贝。
 * 单写单/多读，读者可能读到稍旧的一帧，但不会读到半写状态。
 */
class SnapshotBuffer
{
public:
  /// @brief 发布新快照（仅写线程调用）。
  void publish(const StateSnapshot & snapshot)
  {
    const uint32_t next = 1U - front_.load(std::memory_order_relaxed);
    buffers_[next] = snapshot;
    front_.store(next, std::memory_order_release);
  }

  /// @brief 取当前快照（任意线程可调用）。
  StateSnapshot get() const
  {
    const uint32_t idx = front_.load(std::memory_order_acquire);
    return buffers_[idx];
  }

private:
  std::array<StateSnapshot, 2> buffers_{};
  std::atomic<uint32_t> front_{0};
};

/**
 * @brief 命令槽：控制器回调写，IO 线程取走下发。
 *
 * 覆盖式（只保留最新目标值），不排队；带 priority 让"停机"命令跳过去重。
 */
class CommandSlot
{
public:
  /**
 * @brief 放入一条命令。
 * @param velocity 两轮目标角速度 [rad/s]（已限幅）
 * @param priority true 表示安全命令（如停机），IO 线程必须下发
 */
  void set(const std::array<double, 2> & velocity, bool priority)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    velocity_ = velocity;
    pending_ = true;
    priority_ = priority;
  }

  /**
 * @brief 取走待发命令。
 * @param[out] velocity 目标角速度
 * @return true 表示取到了命令（此时 pending 已清空）
 */
  bool take(std::array<double, 2> & velocity)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!pending_) {
      return false;
    }
    velocity = velocity_;
    pending_ = false;
    priority_ = false;
    return true;
  }

  /// @brief 是否存在安全命令（供 IO 线程判断是否需要特殊处理）。
  bool priority_pending() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return pending_ && priority_;
  }

private:
  mutable std::mutex mutex_;
  std::array<double, 2> velocity_{0.0, 0.0};
  bool pending_{false};
  bool priority_{false};
};

/**
 * @brief 新帧事件：IO 线程递增序号并唤醒 IMU 线程。
 *
 * 用"序号 + 条件变量"而不是定时器，保证不重复发布旧数据、也不错过新数据。
 */
class FrameSignal
{
public:
  /// @brief 标记有新帧（IO 线程调用）。
  void notify()
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      ++sequence_;
    }
    cv_.notify_one();
  }

  /**
 * @brief 等待序号变化。
 * @param last_seen 调用者上次见到的序号（会被更新为新值）
 * @param timeout 最长等待时间，避免线程永久阻塞
 * @return true 序号有变化；false 超时或被唤醒但无新帧
 */
  bool wait_for_new(uint64_t & last_seen, std::chrono::milliseconds timeout)
  {
    std::unique_lock<std::mutex> lock(mutex_);
    const bool changed = cv_.wait_for(lock, timeout, [&] {return sequence_ != last_seen;});
    if (changed) {
      last_seen = sequence_;
    }
    return changed;
  }

  /// @brief 唤醒所有等待者（线程退出前调用，配合停止标志）。
  void wake_all()
  {
    cv_.notify_all();
  }

private:
  std::mutex mutex_;
  std::condition_variable cv_;
  uint64_t sequence_{0};
};

}  // namespace dd_chassis_001_driver

#endif  // DD_CHASSIS_001_DRIVER__DATA_EXCHANGE_HPP_
