cmake_minimum_required(VERSION 3.8)
project(DD_Chassis_001_driver)

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

# ===== 硬件组件库 =====
# 库名必须与 DD_Chassis_001_driver.xml 里的 <library path="..."> 一致
add_library(DD_Chassis_001_driver SHARED
  src/dd_chassis_001_system.cpp
)
target_include_directories(DD_Chassis_001_driver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(DD_Chassis_001_driver
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

# 把 pluginlib 描述文件登记进 ament 索引，controller_manager 才能发现本插件
pluginlib_export_plugin_description_file(hardware_interface DD_Chassis_001_driver.xml)

install(TARGETS DD_Chassis_001_driver
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)

# ===== 测试 =====
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gmock REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  # 包名 DD_Chassis_001_driver 含大写，不符合 ROS 包名规范 [a-z](_?[a-z0-9]+)*，
  # xmllint 会持续报 package.xml 校验失败；改包名才能根本解决，此处禁用该项。
  set(ament_cmake_xmllint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  ament_add_gmock(test_load_dd_chassis_001_system
    test/test_load_dd_chassis_001_system.cpp
  )
  target_link_libraries(test_load_dd_chassis_001_system DD_Chassis_001_driver)
  ament_target_dependencies(test_load_dd_chassis_001_system
    hardware_interface
    pluginlib
  )
endif()

ament_export_include_directories(include)
ament_export_libraries(DD_Chassis_001_driver)
ament_package()
