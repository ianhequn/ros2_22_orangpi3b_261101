/**
 * @file visibility_control.hpp
 * @brief 符号导出宏。
 *
 * ROS2 编译时默认 -fvisibility=hidden，被 pluginlib 动态加载的类必须显式导出，
 * 否则 controller_manager 加载时会报 "Failed to load library / undefined symbol"。
 * 换包名时只需改宏前缀与字符串 DD_CHASSIS_001_DRIVER。
 */
#ifndef DD_CHASSIS_001_DRIVER__VISIBILITY_CONTROL_HPP_
#define DD_CHASSIS_001_DRIVER__VISIBILITY_CONTROL_HPP_

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define DD_CHASSIS_001_DRIVER_EXPORT __attribute__((dllexport))
    #define DD_CHASSIS_001_DRIVER_IMPORT __attribute__((dllimport))
  #else
    #define DD_CHASSIS_001_DRIVER_EXPORT __declspec(dllexport)
    #define DD_CHASSIS_001_DRIVER_IMPORT __declspec(dllimport)
  #endif
  #ifdef DD_CHASSIS_001_DRIVER_BUILDING_LIBRARY
    #define DD_CHASSIS_001_DRIVER_PUBLIC DD_CHASSIS_001_DRIVER_EXPORT
  #else
    #define DD_CHASSIS_001_DRIVER_PUBLIC DD_CHASSIS_001_DRIVER_IMPORT
  #endif
  #define DD_CHASSIS_001_DRIVER_PUBLIC_TYPE DD_CHASSIS_001_DRIVER_PUBLIC
  #define DD_CHASSIS_001_DRIVER_LOCAL
#else
  #define DD_CHASSIS_001_DRIVER_EXPORT __attribute__((visibility("default")))
  #define DD_CHASSIS_001_DRIVER_IMPORT
  #define DD_CHASSIS_001_DRIVER_PUBLIC __attribute__((visibility("default")))
  #define DD_CHASSIS_001_DRIVER_PUBLIC_TYPE
  #define DD_CHASSIS_001_DRIVER_LOCAL __attribute__((visibility("hidden")))
#endif

#endif  // DD_CHASSIS_001_DRIVER__VISIBILITY_CONTROL_HPP_
