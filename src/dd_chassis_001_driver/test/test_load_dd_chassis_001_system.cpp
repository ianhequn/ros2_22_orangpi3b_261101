/**
 * @file test_load_dd_chassis_001_system.cpp
 * @brief 插件可加载性测试：确认 pluginlib 能找到并实例化本硬件组件。
 *
 * 这是成本最低的一道关卡：
 *   编译通过 + 本测试通过  =>  .so 导出正常、xml 名字对得上、类不是抽象类。
 * 运行：colcon test --packages-select dd_chassis_001_driver
 *       colcon test-result --verbose
 */
#include <memory>

#include "gtest/gtest.h"

#include "hardware_interface/system_interface.hpp"
#include "pluginlib/class_loader.hpp"

TEST(TestLoadDDChassis001System, load_plugin)
{
  // 参数1：基类所属的包；参数2：基类类型名
  pluginlib::ClassLoader<hardware_interface::SystemInterface> loader(
    "hardware_interface", "hardware_interface::SystemInterface");

  // 参数3：插件名，必须与 dd_chassis_001_driver.xml 里的 <class name="..."> 一致
  EXPECT_NO_THROW(
  {
    auto system = loader.createSharedInstance(
      "dd_chassis_001_driver/DDChassis001System");
    EXPECT_NE(system, nullptr);
  });
}
