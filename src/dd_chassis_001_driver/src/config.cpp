/**
 * @file config.cpp
 * @brief Config 的解析与校验实现。
 */
#include "dd_chassis_001_driver/config.hpp"

#include <cerrno>
#include <cstdlib>
#include <string>

namespace dd_chassis_001_driver
{
namespace
{
/**
 * @brief 取参数原始字符串。
 * @return true 表示存在（即使为空串）。
 */
bool find_param(
  const std::unordered_map<std::string, std::string> & params,
  const std::string & key, std::string & value)
{
  const auto it = params.find(key);
  if (it == params.end()) {
    return false;
  }
  value = it->second;
  return true;
}

/**
 * @brief 解析 double；成功要求整个字符串都被消费（拒绝 "1.0abc"）。
 */
bool parse_double(const std::string & text, double & out)
{
  if (text.empty()) {
    return false;
  }
  errno = 0;
  char * end = nullptr;
  const double value = std::strtod(text.c_str(), &end);
  if (end == text.c_str() || *end != '\0' || errno == ERANGE) {
    return false;
  }
  out = value;
  return true;
}

/**
 * @brief 解析 int（十进制），同样要求整串消费。
 */
bool parse_int(const std::string & text, int & out)
{
  if (text.empty()) {
    return false;
  }
  errno = 0;
  char * end = nullptr;
  const long value = std::strtol(text.c_str(), &end, 10);
  if (end == text.c_str() || *end != '\0' || errno == ERANGE) {
    return false;
  }
  out = static_cast<int>(value);
  return true;
}

/**
 * @brief 解析 uint32，支持十进制与 0x 前缀十六进制（帧尾魔数用）。
 */
bool parse_u32(const std::string & text, uint32_t & out)
{
  if (text.empty()) {
    return false;
  }
  errno = 0;
  char * end = nullptr;
  const unsigned long value = std::strtoul(text.c_str(), &end, 0);
  if (end == text.c_str() || *end != '\0' || errno == ERANGE || value > 0xFFFFFFFFUL) {
    return false;
  }
  out = static_cast<uint32_t>(value);
  return true;
}

/**
 * @brief 解析布尔，接受 true/false/1/0/yes/no（大小写不敏感）。
 */
bool parse_bool(const std::string & text, bool & out)
{
  if (text == "true" || text == "True" || text == "TRUE" || text == "1" || text == "yes") {
    out = true;
    return true;
  }
  if (text == "false" || text == "False" || text == "FALSE" || text == "0" || text == "no") {
    out = false;
    return true;
  }
  return false;
}

/// 失败时统一填充 error_message 的小工具
ParseResult fail(ParseResult result, const std::string & message, std::string & error_message)
{
  error_message = message;
  return result;
}

/**
 * @brief 读取可选参数：不存在则保持默认值；存在但非法则返回 kInvalid。
 * @tparam T 目标类型
 * @tparam Parser 解析函数类型 void(const std::string &, T &)
 */
template<typename T, typename Parser>
ParseResult read_optional(
  const std::unordered_map<std::string, std::string> & params,
  const std::string & key, T & target, Parser parser, std::string & error_message)
{
  std::string text;
  if (!find_param(params, key, text)) {
    return ParseResult::kOk;      // 可选参数：缺省即默认值
  }
  if (text.empty()) {
    return fail(ParseResult::kInvalid, "参数 \"" + key + "\" 为空", error_message);
  }
  if (!parser(text, target)) {
    return fail(ParseResult::kInvalid, "参数 \"" + key + "\" 值非法: \"" + text + "\"", error_message);
  }
  return ParseResult::kOk;
}

/**
 * @brief 读取必填字符串参数，缺失或空串都算 kMissing。
 */
ParseResult read_required_string(
  const std::unordered_map<std::string, std::string> & params,
  const std::string & key, std::string & target, std::string & error_message)
{
  std::string text;
  if (!find_param(params, key, text) || text.empty()) {
    return fail(ParseResult::kMissing, "缺少必填参数 \"" + key + "\"", error_message);
  }
  target = text;
  return ParseResult::kOk;
}

/**
 * @brief 解析单个轮子的配置块，并做局部校验。
 * @param prefix 参数前缀，如 "left" -> left_wheel_joint / left_motor_index / ...
 */
ParseResult parse_wheel(
  const std::unordered_map<std::string, std::string> & params,
  const std::string & prefix, WheelConfig & wheel, std::string & error_message)
{
  const ParseResult r = read_required_string(
    params, prefix + "_wheel_joint", wheel.joint_name, error_message);
  if (r != ParseResult::kOk) {
    return r;
  }

  ParseResult rc = read_optional(
    params, prefix + "_motor_index", wheel.motor_index,
    [](const std::string & s, int & v) {return parse_int(s, v);}, error_message);
  if (rc != ParseResult::kOk) {
    return rc;
  }
  if (wheel.motor_index != 0 && wheel.motor_index != 1) {
    return fail(
      ParseResult::kInvalid,
      "参数 \"" + prefix + "_motor_index\" 必须为 0 或 1，当前 " +
      std::to_string(wheel.motor_index), error_message);
  }

  rc = read_optional(
    params, prefix + "_speed_scale", wheel.speed_scale,
    [](const std::string & s, double & v) {return parse_double(s, v);}, error_message);
  if (rc != ParseResult::kOk) {
    return rc;
  }
  // speed_scale 作为分母参与下发换算，必须严格大于 0
  if (!(wheel.speed_scale > 0.0)) {
    return fail(
      ParseResult::kInvalid,
      "参数 \"" + prefix + "_speed_scale\" 必须 > 0，当前 " +
      std::to_string(wheel.speed_scale), error_message);
  }

  return read_optional(
    params, prefix + "_invert", wheel.invert,
    [](const std::string & s, bool & v) {return parse_bool(s, v);}, error_message);
}
}  // namespace

ParseResult parse_config(
  const std::unordered_map<std::string, std::string> & params,
  Config & cfg, std::string & error_message)
{
  error_message.clear();

  // ---------- 串口 ----------
  ParseResult r = read_required_string(params, "serial_device", cfg.serial_device, error_message);
  if (r != ParseResult::kOk) {
    return r;
  }

  // frame_end 是帧对齐的唯一依据，必须显式给出且非 0
  std::string frame_end_text;
  if (!find_param(params, "frame_end", frame_end_text) || frame_end_text.empty()) {
    return fail(ParseResult::kMissing, "缺少必填参数 \"frame_end\"（固件帧尾魔数）", error_message);
  }
  if (!parse_u32(frame_end_text, cfg.frame_end)) {
    return fail(
      ParseResult::kInvalid,
      "参数 \"frame_end\" 值非法: \"" + frame_end_text + "\"（支持十进制或 0x 十六进制）",
      error_message);
  }
  if (cfg.frame_end == 0) {
    return fail(ParseResult::kInvalid, "参数 \"frame_end\" 不能为 0", error_message);
  }

  r = read_optional(
    params, "baud_rate", cfg.baud_rate,
    [](const std::string & s, int & v) {return parse_int(s, v);}, error_message);
  if (r != ParseResult::kOk) {
    return r;
  }
  if (cfg.baud_rate <= 0) {
    return fail(
      ParseResult::kInvalid, "参数 \"baud_rate\" 必须 > 0", error_message);
  }

  r = read_optional(
    params, "frame_timeout_ms", cfg.frame_timeout_ms,
    [](const std::string & s, int & v) {return parse_int(s, v);}, error_message);
  if (r != ParseResult::kOk) {
    return r;
  }
  if (cfg.frame_timeout_ms <= 0) {
    return fail(
      ParseResult::kInvalid, "参数 \"frame_timeout_ms\" 必须 > 0", error_message);
  }

  r = read_optional(
    params, "timeout_stops_motors", cfg.timeout_stops_motors,
    [](const std::string & s, bool & v) {return parse_bool(s, v);}, error_message);
  if (r != ParseResult::kOk) {
    return r;
  }

  // ---------- 控制 ----------
  r = read_optional(
    params, "max_wheel_velocity", cfg.max_wheel_velocity,
    [](const std::string & s, double & v) {return parse_double(s, v);}, error_message);
  if (r != ParseResult::kOk) {
    return r;
  }

  r = read_optional(
    params, "command_epsilon", cfg.command_epsilon,
    [](const std::string & s, double & v) {return parse_double(s, v);}, error_message);
  if (r != ParseResult::kOk) {
    return r;
  }
  if (cfg.command_epsilon < 0.0) {
    return fail(
      ParseResult::kInvalid, "参数 \"command_epsilon\" 不能为负", error_message);
  }

  r = read_optional(
    params, "command_repeat_period_ms", cfg.command_repeat_period_ms,
    [](const std::string & s, int & v) {return parse_int(s, v);}, error_message);
  if (r != ParseResult::kOk) {
    return r;
  }
  if (cfg.command_repeat_period_ms < 0) {
    return fail(
      ParseResult::kInvalid, "参数 \"command_repeat_period_ms\" 不能为负", error_message);
  }

  // ---------- 两轮 ----------
  r = parse_wheel(params, "left", cfg.left, error_message);
  if (r != ParseResult::kOk) {
    return r;
  }
  r = parse_wheel(params, "right", cfg.right, error_message);
  if (r != ParseResult::kOk) {
    return r;
  }
  if (cfg.left.joint_name == cfg.right.joint_name) {
    return fail(
      ParseResult::kInvalid, "左右轮 joint 名不能相同: \"" + cfg.left.joint_name + "\"",
      error_message);
  }
  if (cfg.left.motor_index == cfg.right.motor_index) {
    return fail(
      ParseResult::kInvalid, "左右轮 motor_index 不能相同: " +
      std::to_string(cfg.left.motor_index), error_message);
  }

  // ---------- IMU ----------
  r = read_optional(
    params, "publish_imu", cfg.publish_imu,
    [](const std::string & s, bool & v) {return parse_bool(s, v);}, error_message);
  if (r != ParseResult::kOk) {
    return r;
  }

  r = read_optional(
    params, "imu_topic", cfg.imu_topic,
    [](const std::string & s, std::string & v) {v = s; return !s.empty();}, error_message);
  if (r != ParseResult::kOk) {
    return r;
  }

  r = read_optional(
    params, "imu_frame_id", cfg.imu_frame_id,
    [](const std::string & s, std::string & v) {v = s; return !s.empty();}, error_message);
  if (r != ParseResult::kOk) {
    return r;
  }

  r = read_optional(
    params, "imu_yaw_field", cfg.imu_yaw_field,
    [](const std::string & s, int & v) {return parse_int(s, v);}, error_message);
  if (r != ParseResult::kOk) {
    return r;
  }
  if (cfg.imu_yaw_field < 1 || cfg.imu_yaw_field > 3) {
    return fail(
      ParseResult::kInvalid,
      "参数 \"imu_yaw_field\" 必须为 1~3（对应 imu1/imu2/imu3），当前 " +
      std::to_string(cfg.imu_yaw_field), error_message);
  }

  r = read_optional(
    params, "imu_angle_is_deg", cfg.imu_angle_is_deg,
    [](const std::string & s, bool & v) {return parse_bool(s, v);}, error_message);
  if (r != ParseResult::kOk) {
    return r;
  }

  r = read_optional(
    params, "imu_yaw_sign", cfg.imu_yaw_sign,
    [](const std::string & s, double & v) {return parse_double(s, v);}, error_message);
  if (r != ParseResult::kOk) {
    return r;
  }
  if (cfg.imu_yaw_sign != 1.0 && cfg.imu_yaw_sign != -1.0) {
    return fail(
      ParseResult::kInvalid,
      "参数 \"imu_yaw_sign\" 只能是 1 或 -1，当前 " + std::to_string(cfg.imu_yaw_sign),
      error_message);
  }

  r = read_optional(
    params, "imu_publish_rate", cfg.imu_publish_rate,
    [](const std::string & s, double & v) {return parse_double(s, v);}, error_message);
  if (r != ParseResult::kOk) {
    return r;
  }
  if (!(cfg.imu_publish_rate > 0.0)) {
    return fail(
      ParseResult::kInvalid, "参数 \"imu_publish_rate\" 必须 > 0", error_message);
  }

  r = read_optional(
    params, "imu_stamp_offset_calib_s", cfg.imu_stamp_offset_calib_s,
    [](const std::string & s, double & v) {return parse_double(s, v);}, error_message);
  if (r != ParseResult::kOk) {
    return r;
  }
  if (cfg.imu_stamp_offset_calib_s < 0.0) {
    return fail(
      ParseResult::kInvalid, "参数 \"imu_stamp_offset_calib_s\" 不能为负", error_message);
  }

  // ---------- 调试 ----------
  r = read_optional(
    params, "debug_log", cfg.debug_log,
    [](const std::string & s, bool & v) {return parse_bool(s, v);}, error_message);
  if (r != ParseResult::kOk) {
    return r;
  }

  return ParseResult::kOk;
}

}  // namespace dd_chassis_001_driver
