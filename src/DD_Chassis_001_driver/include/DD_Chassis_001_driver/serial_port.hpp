#pragma once

#include <cstddef>
#include <cstdint>
#include <string>

namespace dd_chassis_001
{
/// 串口封装：只负责 打开 / 非阻塞读 / 写，不含 ROS 与协议逻辑。
class SerialPort
{
public:
  SerialPort() = default;
  ~SerialPort();
  SerialPort(const SerialPort &) = delete;
  SerialPort & operator=(const SerialPort &) = delete;

  /// 打开并配置为 raw 8N1 非阻塞模式；失败时 error 写入原因
  bool open(const std::string & device, int baud_rate, std::string & error);

  void close();

  bool is_open() const {return fd_ >= 0;}

  /// 非阻塞读：返回读到的字节数，0 = 当前无数据，-1 = 设备错误
  ssize_t read(uint8_t * dst, size_t max_len);

  /// 写全部字节（命令都很短），返回是否全部写出
  bool write(const std::string & data);

private:
  int fd_ = -1;
};

}  // namespace dd_chassis_001
