cmake_minimum_required(VERSION 3.8)
project(dd_chassis_001_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ===== 依赖 =====
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)

# ===== 硬件组件库 =====
# 库名必须与 dd_chassis_001_driver.xml 里的 <library path="..."> 一致
add_library(dd_chassis_001_driver SHARED
  src/dd_chassis_001_system.cpp
  src/config.cpp
  src/serial_port.cpp
  src/chassis_protocol.cpp
)
target_include_directories(dd_chassis_001_driver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(dd_chassis_001_driver
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  sensor_msgs
)

# 把 pluginlib 描述文件登记进 ament 索引，controller_manager 才能发现本插件
pluginlib_export_plugin_description_file(hardware_interface dd_chassis_001_driver.xml)

install(TARGETS dd_chassis_001_driver
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)

install(DIRECTORY config launch note DESTINATION share/${PROJECT_NAME})
install(PROGRAMS
  scripts/serial_monitor.py
  scripts/chassis_monitor.py
  DESTINATION lib/${PROJECT_NAME}
)

# ===== 测试 =====
# 本仓库（orangepi）不保留测试：单测与 lint 只在 WSL 侧仓库维护，
# 免得真机环境还要装 ament-cmake-gmock / ament-lint-common。
ament_export_include_directories(include)
ament_export_libraries(dd_chassis_001_driver)
ament_package()
