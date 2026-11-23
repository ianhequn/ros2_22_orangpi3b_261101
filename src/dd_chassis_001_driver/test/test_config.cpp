/**
 * @file test_config.cpp
 * @brief 参数解析与校验的单元测试。
 *
 * 覆盖三件事：
 *  1. 完整参数能正确解析成 Config；
 *  2. 必填项缺失返回 kMissing，非法值返回 kInvalid；
 *  3. 交叉约束（左右轮重名/同电机号等）能拦住。
 */
#include <string>
#include <unordered_map>

#include "gtest/gtest.h"

#include "dd_chassis_001_driver/config.hpp"

namespace
{
using ParamMap = std::unordered_map<std::string, std::string>;

/// 构造一份最小可用的合法参数集（各用例在此基础上改）
ParamMap make_valid_params()
{
  return ParamMap{
    {"serial_device", "/dev/ttyUSB1"},
    {"baud_rate", "1000000"},
    {"frame_end", "0x7F800000"},
    {"left_wheel_joint", "wheel_l"},
    {"right_wheel_joint", "wheel_r"},
  };
}
}  // namespace

TEST(TestConfig, parses_full_param_set)
{
  ParamMap params = make_valid_params();
  params["frame_timeout_ms"] = "300";
  params["left_motor_index"] = "0";
  params["right_motor_index"] = "1";
  params["left_speed_scale"] = "0.01";
  params["right_speed_scale"] = "0.02";
  params["left_invert"] = "true";
  params["max_wheel_velocity"] = "20.0";
  params["imu_yaw_field"] = "3";
  params["imu_angle_is_deg"] = "false";
  params["imu_yaw_sign"] = "-1";
  params["debug_log"] = "true";

  dd_chassis_001_driver::Config cfg;
  std::string err;
  ASSERT_EQ(
    dd_chassis_001_driver::parse_config(params, cfg, err),
    dd_chassis_001_driver::ParseResult::kOk) << err;

  EXPECT_EQ(cfg.serial_device, "/dev/ttyUSB1");
  EXPECT_EQ(cfg.baud_rate, 1000000);
  EXPECT_EQ(cfg.frame_end, 0x7F800000u);
  EXPECT_EQ(cfg.frame_timeout_ms, 300);
  EXPECT_EQ(cfg.left.joint_name, "wheel_l");
  EXPECT_EQ(cfg.left.motor_index, 0);
  EXPECT_DOUBLE_EQ(cfg.left.speed_scale, 0.01);
  EXPECT_TRUE(cfg.left.invert);
  EXPECT_DOUBLE_EQ(cfg.right.speed_scale, 0.02);
  EXPECT_FALSE(cfg.right.invert);
  EXPECT_DOUBLE_EQ(cfg.max_wheel_velocity, 20.0);
  EXPECT_EQ(cfg.imu_yaw_field, 3);
  EXPECT_FALSE(cfg.imu_angle_is_deg);
  EXPECT_DOUBLE_EQ(cfg.imu_yaw_sign, -1.0);
  EXPECT_TRUE(cfg.debug_log);
}

TEST(TestConfig, missing_frame_end_is_reported_as_missing)
{
  ParamMap params = make_valid_params();
  params.erase("frame_end");

  dd_chassis_001_driver::Config cfg;
  std::string err;
  EXPECT_EQ(
    dd_chassis_001_driver::parse_config(params, cfg, err),
    dd_chassis_001_driver::ParseResult::kMissing);
  EXPECT_NE(err.find("frame_end"), std::string::npos);
}

TEST(TestConfig, invalid_baud_rate_is_reported_as_invalid)
{
  ParamMap params = make_valid_params();
  params["baud_rate"] = "abc";

  dd_chassis_001_driver::Config cfg;
  std::string err;
  EXPECT_EQ(
    dd_chassis_001_driver::parse_config(params, cfg, err),
    dd_chassis_001_driver::ParseResult::kInvalid);
  EXPECT_NE(err.find("baud_rate"), std::string::npos);
}

TEST(TestConfig, trailing_garbage_in_number_is_rejected)
{
  ParamMap params = make_valid_params();
  params["frame_end"] = "0x7F800000xyz";

  dd_chassis_001_driver::Config cfg;
  std::string err;
  EXPECT_EQ(
    dd_chassis_001_driver::parse_config(params, cfg, err),
    dd_chassis_001_driver::ParseResult::kInvalid);
}

TEST(TestConfig, zero_speed_scale_is_rejected)
{
  ParamMap params = make_valid_params();
  params["left_speed_scale"] = "0";

  dd_chassis_001_driver::Config cfg;
  std::string err;
  EXPECT_EQ(
    dd_chassis_001_driver::parse_config(params, cfg, err),
    dd_chassis_001_driver::ParseResult::kInvalid);
  EXPECT_NE(err.find("left_speed_scale"), std::string::npos);
}

TEST(TestConfig, imu_yaw_field_out_of_range_is_rejected)
{
  ParamMap params = make_valid_params();
  params["imu_yaw_field"] = "4";

  dd_chassis_001_driver::Config cfg;
  std::string err;
  EXPECT_EQ(
    dd_chassis_001_driver::parse_config(params, cfg, err),
    dd_chassis_001_driver::ParseResult::kInvalid);
}

TEST(TestConfig, duplicate_motor_index_is_rejected)
{
  ParamMap params = make_valid_params();
  params["left_motor_index"] = "1";
  params["right_motor_index"] = "1";

  dd_chassis_001_driver::Config cfg;
  std::string err;
  EXPECT_EQ(
    dd_chassis_001_driver::parse_config(params, cfg, err),
    dd_chassis_001_driver::ParseResult::kInvalid);
  EXPECT_NE(err.find("motor_index"), std::string::npos);
}

TEST(TestConfig, defaults_are_used_when_optionals_absent)
{
  ParamMap params = make_valid_params();

  dd_chassis_001_driver::Config cfg;
  std::string err;
  ASSERT_EQ(
    dd_chassis_001_driver::parse_config(params, cfg, err),
    dd_chassis_001_driver::ParseResult::kOk) << err;

  // 未给出的可选参数应保持结构体默认值
  EXPECT_EQ(cfg.left.motor_index, 0);
  EXPECT_DOUBLE_EQ(cfg.left.speed_scale, 1.0);
  EXPECT_FALSE(cfg.left.invert);
  EXPECT_TRUE(cfg.timeout_stops_motors);
  EXPECT_EQ(cfg.imu_yaw_field, 1);
  EXPECT_TRUE(cfg.imu_angle_is_deg);
  EXPECT_FALSE(cfg.debug_log);
}
