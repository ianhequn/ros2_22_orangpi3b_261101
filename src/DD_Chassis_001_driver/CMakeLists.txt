cmake_minimum_required(VERSION 3.8)
project(DD_Chassis_001_driver)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_library(dd_chassis_001_hardware SHARED
  src/serial_port.cpp
  src/chassis_protocol.cpp
  src/dd_chassis_001_system.cpp
)
target_include_directories(dd_chassis_001_hardware PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(dd_chassis_001_hardware
  hardware_interface
  pluginlib
  rclcpp
  sensor_msgs
)

pluginlib_export_plugin_description_file(hardware_interface dd_chassis_001_hardware.xml)

install(TARGETS dd_chassis_001_hardware
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(dd_chassis_001_hardware)
ament_package()
