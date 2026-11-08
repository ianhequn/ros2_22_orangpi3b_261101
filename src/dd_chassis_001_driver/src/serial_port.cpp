/**
 * @file serial_port.cpp
 * @brief SerialPort 实现（termios）。
 */
#include "dd_chassis_001_driver/serial_port.hpp"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>

namespace dd_chassis_001_driver
{
namespace
{
/**
 * @brief 整数波特率 → termios speed_t。
 *
 * 表里没有的值会导致 tcsetattr 行为未定义，所以宁可显式失败。
 */
bool to_termios_baud(int baud_rate, speed_t & out)
{
  switch (baud_rate) {
    case 1200: out = B1200; return true;
    case 2400: out = B2400; return true;
    case 4800: out = B4800; return true;
    case 9600: out = B9600; return true;
    case 19200: out = B19200; return true;
    case 38400: out = B38400; return true;
    case 57600: out = B57600; return true;
    case 115200: out = B115200; return true;
    case 230400: out = B230400; return true;
    case 460800: out = B460800; return true;
    case 500000: out = B500000; return true;
    case 576000: out = B576000; return true;
    case 921600: out = B921600; return true;
    case 1000000: out = B1000000; return true;
    case 1152000: out = B1152000; return true;
    case 1500000: out = B1500000; return true;
    case 2000000: out = B2000000; return true;
    case 2500000: out = B2500000; return true;
    case 3000000: out = B3000000; return true;
    case 3500000: out = B3500000; return true;
    case 4000000: out = B4000000; return true;
    default: return false;
  }
}
}  // namespace

SerialPort::~SerialPort()
{
  close();
}

bool SerialPort::open(const std::string & device, int baud_rate, std::string & error)
{
  close();

  speed_t baud = B0;
  if (!to_termios_baud(baud_rate, baud)) {
    error = "不支持的波特率: " + std::to_string(baud_rate);
    return false;
  }

  // O_NONBLOCK：读不会阻塞调用方；O_NOCTTY：不把串口当控制终端
  fd_ = ::open(device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
  if (fd_ < 0) {
    error = "打开串口失败 " + device + ": " + std::strerror(errno);
    return false;
  }

  struct termios tio {};
  if (tcgetattr(fd_, &tio) != 0) {
    error = "tcgetattr 失败: " + std::string(std::strerror(errno));
    close();
    return false;
  }

  cfmakeraw(&tio);                  // 关闭行缓冲/回显/特殊字符，纯二进制透明传输
  tio.c_cflag |= (CLOCAL | CREAD);  // 忽略调制解调器控制线，允许接收
  tio.c_cflag &= ~CRTSCTS;          // 无硬件流控
  tio.c_cc[VMIN] = 0;               // 读不等待最少字节数
  tio.c_cc[VTIME] = 0;              // 读无超时 -> 配合 O_NONBLOCK 实现"有多少读多少"
  cfsetispeed(&tio, baud);
  cfsetospeed(&tio, baud);

  if (tcsetattr(fd_, TCSANOW, &tio) != 0) {
    error = "tcsetattr 失败: " + std::string(std::strerror(errno));
    close();
    return false;
  }

  tcflush(fd_, TCIOFLUSH);  // 丢掉打开前残留的脏数据
  return true;
}

void SerialPort::close()
{
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

ssize_t SerialPort::read(uint8_t * dst, size_t max_len)
{
  if (fd_ < 0) {
    return -1;
  }
  const ssize_t n = ::read(fd_, dst, max_len);
  if (n < 0) {
    if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) {
      return 0;  // 暂无数据，不是错误
    }
    return -1;
  }
  return n;
}

bool SerialPort::write(const std::string & data)
{
  if (fd_ < 0) {
    return false;
  }

  size_t sent = 0;
  int retries = 0;
  while (sent < data.size()) {
    const ssize_t n = ::write(fd_, data.data() + sent, data.size() - sent);
    if (n < 0) {
      if (errno == EINTR) {
        continue;
      }
      if (errno == EAGAIN || errno == EWOULDBLOCK) {
        // 发送缓冲暂满：命令很短，短暂重试即可（不允许在此长时间阻塞 IO 线程）
        if (++retries > 1000) {
          return false;
        }
        continue;
      }
      return false;
    }
    sent += static_cast<size_t>(n);
  }
  return true;
}

}  // namespace dd_chassis_001_driver
