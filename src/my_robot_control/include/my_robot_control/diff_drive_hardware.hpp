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

// 差速底盘硬件接口。
//
// 生命周期：on_init -> on_configure -> on_activate -> (read/write 循环) -> on_deactivate -> on_cleanup
//    on_init      只调一次，负责读 <ros2_control> 里的参数
//    on_configure 可反复调，负责"打开设备"（这里分配缓冲）
//    on_activate  上电（清状态）
//    on_cleanup   关设备（释放）
//    on_error     出错时复位
class DiffDriveHardware : public hardware_interface::SystemInterface
{
public:
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_error(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // ── 状态（暴露给框架的，必须是类成员：框架持有它们的地址）──
  std::vector<double> position_;      // 编码器累计位置（已量化，rad）
  std::vector<double> velocity_;      // 轮速（rad/s）
  std::vector<double> effort_;        // 力矩反馈（N·m）

  // ── 指令 ──
  std::vector<double> velocity_command_;   // rad/s
  std::vector<double> effort_command_;     // N·m

  // ── 内部量（不暴露）──
  std::vector<double> position_raw_;       // 连续位置，量化前

  // ── 参数（从 <ros2_control> 的 <param> 读）──
  std::string motor_mode_{"velocity"};     // velocity | effort
  double motor_inertia_{0.5};              // 转动惯量 J (kg·m²)
  double viscous_friction_{0.05};          // 粘滞摩擦系数 b (N·m·s/rad)
  double vel_gain_{2.0};                   // 速度环 P 增益
  double max_effort_{5.0};                 // 力矩上限
  double max_wheel_speed_{10.0};           // 轮速上限
  double encoder_cpr_{4096.0};             // 编码器线数（每圈 4096）
  double wheel_separation_{0.30};          // 轮距，用于算真实偏航
  double wheel_radius_{0.05};

  bool configured_{false};
};

}  // namespace my_robot_control

#endif  // MY_ROBOT_CONTROL__DIFF_DRIVE_HARDWARE_HPP_
