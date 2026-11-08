/**
 * @file dd_chassis_001_system.cpp
 * @brief DDChassis001System 实现骨架。
 *
 * 建议的填充顺序（每一步都能独立验证）：
 *  1. on_init      : 参数解析 + 校验（不碰硬件）
 *  2. export_*     : 先暴露接口，用假数据让控制器跑起来
 *  3. on_configure : 打开串口
 *  4. read         : 帧解析 -> wheel_velocity_ / wheel_position_
 *  5. write        : 命令换算 -> 下发速度指令
 *  6. on_activate / on_deactivate / on_shutdown / on_error : 使能/停机/收尾
 */

#include "DD_Chassis_001_driver/dd_chassis_001_system.hpp"

#include <string>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace dd_chassis_001_driver
{
    namespace
    {
        /**
         * @brief 从硬件参数表读取字符串参数，不存在时返回 fallback。
         * @param info 硬件描述
         * @param key  参数名（URDF <param name="...">）
         * @param fallback 缺省值
         */
        std::string get_param(
            const hardware_interface::HardwareInfo &info, const std::string &key,
            const std::string &fallback)
        {
            const auto it = info.hardware_parameters.find(key);
            return it == info.hardware_parameters.end() ? fallback : it->second;
        }
    } // namespace

    hardware_interface::CallbackReturn DDChassis001System::on_init(
        const hardware_interface::HardwareInfo &info)
    {
        // 先让父类解析 name/type 等标准字段
        if (hardware_interface::SystemInterface::on_init(info) !=
            hardware_interface::CallbackReturn::SUCCESS)
        {
            return hardware_interface::CallbackReturn::ERROR;
        }

        serial_device_ = get_param(info, "serial_device", serial_device_);
        left_joint_ = get_param(info, "left_wheel_joint", left_joint_);
        right_joint_ = get_param(info, "right_wheel_joint", right_joint_);
        // TODO: 解析 baud_rate / frame_end / speed_scale / invert / 限幅 / 超时
        //       必填项缺失或非法：RCLCPP_ERROR 后 return ERROR

        // TODO: 校验 info.joints 中确实存在 left_joint_ / right_joint_

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn DDChassis001System::on_configure(
        const rclcpp_lifecycle::State & /*previous_state*/)
    {
        // TODO: 打开串口（8N1、raw、O_NONBLOCK + VMIN=0/VTIME=0），失败 return ERROR
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn DDChassis001System::on_cleanup(
        const rclcpp_lifecycle::State & /*previous_state*/)
    {
        // TODO: 关闭串口、清空接收缓冲
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn DDChassis001System::on_shutdown(
        const rclcpp_lifecycle::State & /*previous_state*/)
    {
        // TODO: 给固件下发 0 速度，再释放资源
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn DDChassis001System::on_activate(
        const rclcpp_lifecycle::State & /*previous_state*/)
    {
        // TODO: 电机使能/清故障；把 wheel_position_ 等积分量归零
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn DDChassis001System::on_deactivate(
        const rclcpp_lifecycle::State & /*previous_state*/)
    {
        // TODO: 停机：给固件下发 0 速度（安全相关，不能漏）
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn DDChassis001System::on_error(
        const rclcpp_lifecycle::State & /*previous_state*/)
    {
        // TODO: error 状态兜底，至少停机
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface> DDChassis001System::export_state_interfaces()
    {
        std::vector<hardware_interface::StateInterface> state_interfaces;
        // 接口名格式固定为 "<joint_name>/<interface_name>"
        state_interfaces.emplace_back(
            left_joint_, hardware_interface::HW_IF_POSITION, &wheel_position_[0]);
        state_interfaces.emplace_back(
            left_joint_, hardware_interface::HW_IF_VELOCITY, &wheel_velocity_[0]);
        state_interfaces.emplace_back(
            right_joint_, hardware_interface::HW_IF_POSITION, &wheel_position_[1]);
        state_interfaces.emplace_back(
            right_joint_, hardware_interface::HW_IF_VELOCITY, &wheel_velocity_[1]);
        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface> DDChassis001System::export_command_interfaces()
    {
        std::vector<hardware_interface::CommandInterface> command_interfaces;
        command_interfaces.emplace_back(
            left_joint_, hardware_interface::HW_IF_VELOCITY, &wheel_velocity_cmd_[0]);
        command_interfaces.emplace_back(
            right_joint_, hardware_interface::HW_IF_VELOCITY, &wheel_velocity_cmd_[1]);
        return command_interfaces;
    }

    hardware_interface::return_type DDChassis001System::read(
        const rclcpp::Time & /*time*/, const rclcpp::Duration &period)
    {
        // TODO: 1) 非阻塞读串口，一次读干净所有积压字节
        //       2) 帧解析（无帧头，靠帧尾魔数对齐；一次可能解析出多帧）
        //       3) 固件 speed 值 * 换算系数 -> wheel_velocity_（方向差异另用 invert 处理）
        //       4) wheel_position_ += wheel_velocity_ * period.seconds()（仅通信正常时积分）
        //       5) 超时无帧：上报 0 + 限流告警，不要轻易返回 ERROR
        (void)period;
        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type DDChassis001System::write(
        const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
    {
        // TODO: 1) 命令限幅
        //       2) 关节速度 -> 固件值：固件值 = 关节速度 / 换算系数（方向 invert 再取反）
        //       3) 与上次下发值比较，变化超过阈值才发（固件会保持上次目标）
        //          若固件有指令超时保护，则按固定周期重发
        return hardware_interface::return_type::OK;
    }

} // namespace dd_chassis_001_driver

// 导出给 pluginlib：第一个参数必须与 xml 中 <class type="..."> 一致
PLUGINLIB_EXPORT_CLASS(
    dd_chassis_001_driver::DDChassis001System, hardware_interface::SystemInterface)
