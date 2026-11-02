
#ifndef MY_ROBOT_CONTROL__DIFF_DRIVE_HARDWARE_HPP_
#define MY_ROBOT_CONTROL__DIFF_DRIVE_HARDWARE_HPP_

#include <string>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace my_robot_control
{

class DiffDriveHardware
    : public hardware_interface::SystemInterface
{
public:

    // 初始化硬件
    hardware_interface::CallbackReturn on_init(
        const hardware_interface::HardwareInfo & info) override;

    // 导出状态接口
    std::vector<hardware_interface::StateInterface>
    export_state_interfaces() override;

    // 导出控制接口
    std::vector<hardware_interface::CommandInterface>
    export_command_interfaces() override;

    // 激活硬件
    hardware_interface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State & previous_state) override;

    // 停用硬件
    hardware_interface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State & previous_state) override;

    // 读取硬件状态
    hardware_interface::return_type read(
        const rclcpp::Time & time,
        const rclcpp::Duration & period) override;

    // 写入控制指令
    hardware_interface::return_type write(
        const rclcpp::Time & time,
        const rclcpp::Duration & period) override;

private:

    // 左右轮编码器累计位置
    std::vector<double> position_;

    // 左右轮当前速度
    std::vector<double> velocity_;

    // 左右轮目标速度
    std::vector<double> velocity_command_;

};

}  // namespace my_robot_control

#endif  // MY_ROBOT_CONTROL__DIFF_DRIVE_HARDWARE_HPP_
