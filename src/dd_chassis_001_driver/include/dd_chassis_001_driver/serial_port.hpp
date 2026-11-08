/**
 * @file serial_port.hpp
 * @brief 串口封装：打开 / 非阻塞读 / 写 / 暴露 fd 供 poll 使用。
 *
 * 纯 C++，不依赖 ROS。所有方法不抛异常，错误通过返回值 + error 字符串报告。
 */
#ifndef DD_CHASSIS_001_DRIVER__SERIAL_PORT_HPP_
#define DD_CHASSIS_001_DRIVER__SERIAL_PORT_HPP_

#include <cstddef>
#include <cstdint>
#include <string>

namespace dd_chassis_001_driver
{

/**
 * @brief Linux termios 串口的最小封装（raw 8N1、无流控、非阻塞）。
 */
class SerialPort
{
public:
  SerialPort() = default;
  ~SerialPort();

  SerialPort(const SerialPort &) = delete;
  SerialPort & operator=(const SerialPort &) = delete;

  /**
 * @brief 打开并配置串口。
 * @param device 设备路径，如 /dev/ttyUSB1
 * @param baud_rate 波特率（必须在内部映射表中，否则失败）
 * @param[out] error 失败原因
 * @return 是否成功
 */
  bool open(const std::string & device, int baud_rate, std::string & error);

  /// @brief 关闭串口（幂等）。
  void close();

  /// @brief 是否已打开。
  bool is_open() const {return fd_ >= 0;}

  /// @brief 原始 fd，供 poll() 使用；未打开时为 -1。
  int fd() const {return fd_;}

  /**
 * @brief 非阻塞读。
 * @return >0 读到的字节数；0 表示当前无数据（EAGAIN）；-1 表示设备错误。
 */
  ssize_t read(uint8_t * dst, size_t max_len);

  /**
 * @brief 写完整块数据（处理 EINTR；发送缓冲满时短暂重试）。
 * @return 是否全部写出
 */
  bool write(const std::string & data);

private:
  int fd_{-1};
};

}  // namespace dd_chassis_001_driver

#endif  // DD_CHASSIS_001_DRIVER__SERIAL_PORT_HPP_
