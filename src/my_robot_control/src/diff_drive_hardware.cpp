
#include "my_robot_control/diff_drive_hardware.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>

#include "pluginlib/class_list_macros.hpp"

namespace my_robot_control
{

// 1. 初始化硬件
hardware_interface::CallbackReturn
DiffDriveHardware::on_init(
    const hardware_interface::HardwareInfo & info)
{
    // 先调用父类初始化
    if (hardware_interface::SystemInterface::on_init(info)
        != hardware_interface::CallbackReturn::SUCCESS)
    {
        return hardware_interface::CallbackReturn::ERROR;
    }

    // 当前示例只支持两个轮子
    if (info_.joints.size() != 2)
    {
        return hardware_interface::CallbackReturn::ERROR;
    }

    const std::size_t joint_count = info_.joints.size();

    position_.assign(joint_count, 0.0);
    velocity_.assign(joint_count, 0.0);
    velocity_command_.assign(joint_count, 0.0);

    return hardware_interface::CallbackReturn::SUCCESS;
}


// 2. 导出状态接口
std::vector<hardware_interface::StateInterface>
DiffDriveHardware::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> state_interfaces;

    for (std::size_t i = 0; i < info_.joints.size(); ++i)
    {
        const auto & joint_name = info_.joints[i].name;

        // 编码器累计位置
        state_interfaces.emplace_back(
            joint_name,
            hardware_interface::HW_IF_POSITION,
            &position_[i]);

        // 电机当前速度
        state_interfaces.emplace_back(
            joint_name,
            hardware_interface::HW_IF_VELOCITY,
            &velocity_[i]);
    }

    return state_interfaces;
}


// 3. 导出控制接口
std::vector<hardware_interface::CommandInterface>
DiffDriveHardware::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> command_interfaces;

    for (std::size_t i = 0; i < info_.joints.size(); ++i)
    {
        const auto & joint_name = info_.joints[i].name;

        // 电机速度控制接口
        command_interfaces.emplace_back(
            joint_name,
            hardware_interface::HW_IF_VELOCITY,
            &velocity_command_[i]);
    }

    return command_interfaces;
}


// 4. 激活硬件
hardware_interface::CallbackReturn
DiffDriveHardware::on_activate(
    const rclcpp_lifecycle::State & previous_state)
{
    (void)previous_state;

    std::fill(position_.begin(), position_.end(), 0.0);
    std::fill(velocity_.begin(), velocity_.end(), 0.0);
    std::fill(velocity_command_.begin(), velocity_command_.end(), 0.0);

    return hardware_interface::CallbackReturn::SUCCESS;
}


// 5. 停用硬件
hardware_interface::CallbackReturn
DiffDriveHardware::on_deactivate(
    const rclcpp_lifecycle::State & previous_state)
{
    (void)previous_state;

    std::fill(velocity_.begin(), velocity_.end(), 0.0);
    std::fill(velocity_command_.begin(), velocity_command_.end(), 0.0);

    return hardware_interface::CallbackReturn::SUCCESS;
}


// 6. 读取硬件状态
hardware_interface::return_type
DiffDriveHardware::read(
    const rclcpp::Time & time,
    const rclcpp::Duration & period)
{
    (void)time;

    // 模拟编码器：角度 = 角速度 × 时间
    const double dt = period.seconds();

    for (std::size_t i = 0; i < position_.size(); ++i)
    {
        position_[i] += velocity_[i] * dt;
    }

    return hardware_interface::return_type::OK;
}


// 7. 写入控制指令
hardware_interface::return_type
DiffDriveHardware::write(
    const rclcpp::Time & time,
    const rclcpp::Duration & period)
{
    (void)time;
    (void)period;

    // 模拟电机：当前速度直接跟随目标速度
    constexpr double MAX_WHEEL_SPEED = 10.0;

    for (std::size_t i = 0; i < velocity_.size(); ++i)
    {
        velocity_[i] = std::clamp(
            velocity_command_[i],
            -MAX_WHEEL_SPEED,
            MAX_WHEEL_SPEED);
    }

    return hardware_interface::return_type::OK;
}

}  // namespace my_robot_control


PLUGINLIB_EXPORT_CLASS(
    my_robot_control::DiffDriveHardware,
    hardware_interface::SystemInterface)