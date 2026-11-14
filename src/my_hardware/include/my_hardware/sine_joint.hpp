#ifndef MY_HARDWARE__SINE_JOINT_HPP_
#define MY_HARDWARE__SINE_JOINT_HPP_

#include <chrono>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace my_hardware
{

class SineJoint : public hardware_interface::SystemInterface
{
public:
  // ── 生命周期回调（全部由 controller_manager 调用，用户代码不调用）──
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

  hardware_interface::CallbackReturn on_shutdown(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_error(
    const rclcpp_lifecycle::State & previous_state) override;

  // ── 接口导出（初始化时调一次）──
  std::vector<hardware_interface::StateInterface>
  export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface>
  export_command_interfaces() override;

  // ── 周期回调（100 Hz，由 RT 线程调用）──
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  /// 统一的 [LIFECYCLE] 日志格式，便于观察调用顺序
  void logTransition(const char * cb, const rclcpp_lifecycle::State & prev);

  double pos_{0.0};        ///< 状态：硬件实际位置（由 export_state_interfaces 交出地址）
  double cmd_pos_{0.0};    ///< 命令：控制器写入的目标位置（由 export_command_interfaces 交出地址）
  double amp_{1.0};
  double freq_hz_{0.5};

  std::chrono::steady_clock::time_point t0_;
  bool started_{false};

  /// 调用计数：用来观察 read/write 是否在 100 Hz 上跑
  unsigned long read_count_{0};
  unsigned long write_count_{0};
};

}  // namespace my_hardware

#endif  // MY_HARDWARE__SINE_JOINT_HPP_