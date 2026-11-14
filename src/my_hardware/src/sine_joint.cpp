#include "my_hardware/sine_joint.hpp"

#include <cmath>
#include <string>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace my_hardware
{
using hardware_interface::CallbackReturn;
using hardware_interface::return_type;

// ─────────────────────────────────────────────────────────────
// 统一的 [LIFECYCLE] 日志
//   打印"哪个回调"+"从哪个状态进来"
//   预期顺序：
//     启动:  on_init -> on_configure(prev=unconfigured) -> on_activate(prev=inactive)
//     退出:  on_deactivate(prev=active) -> on_cleanup(prev=inactive)
// ─────────────────────────────────────────────────────────────
void SineJoint::logTransition(const char * cb, const rclcpp_lifecycle::State & prev)
{
  RCLCPP_INFO(
    rclcpp::get_logger("SineJoint"),
    "[LIFECYCLE] %-14s previous_state=%s", cb, prev.label().c_str());
}

CallbackReturn SineJoint::on_init(const hardware_interface::HardwareInfo & info)
{
  RCLCPP_INFO(rclcpp::get_logger("SineJoint"), "[LIFECYCLE] %-14s 开始解析 URDF", "on_init");

  // 必须先调基类：它会校验 <ros2_control> 结构并把 info 存进 info_
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
    RCLCPP_ERROR(rclcpp::get_logger("SineJoint"), "基类 on_init 失败");
    return CallbackReturn::ERROR;
  }

  // info_.hardware_parameters 就是 URDF <hardware> 下的 <param name="...">
  if (info_.hardware_parameters.count("amplitude")) {
    amp_ = std::stod(info_.hardware_parameters.at("amplitude"));
  }
  if (info_.hardware_parameters.count("frequency")) {
    freq_hz_ = std::stod(info_.hardware_parameters.at("frequency"));
  }

  if (info_.hardware_parameters.count("tau")) {
    tau_ = std::stod(info_.hardware_parameters.at("tau"));
  }
  if (info_.hardware_parameters.count("max_pos")) {
    max_pos_ = std::stod(info_.hardware_parameters.at("max_pos"));
  }
  if (info_.hardware_parameters.count("max_vel")) {
    max_vel_ = std::stod(info_.hardware_parameters.at("max_vel"));
  }


  if (info_.joints.empty()) {
    RCLCPP_ERROR(rclcpp::get_logger("SineJoint"), "URDF 里没有声明任何 <joint>");
    return CallbackReturn::ERROR;
  }

  RCLCPP_INFO(
    rclcpp::get_logger("SineJoint"),
    "            joint=%s  amplitude=%.2f  frequency=%.2f Hz",
    info_.joints[0].name.c_str(), amp_, freq_hz_);
  
  RCLCPP_INFO(
    rclcpp::get_logger("SineJoint"),
    "            tau=%.3f s  max_pos=%.2f rad  max_vel=%.1f rad/s",
    tau_, max_pos_, max_vel_);

  return CallbackReturn::SUCCESS;
}

CallbackReturn SineJoint::on_configure(const rclcpp_lifecycle::State & previous_state)
{
  logTransition("on_configure", previous_state);
  return CallbackReturn::SUCCESS;
}

CallbackReturn SineJoint::on_activate(const rclcpp_lifecycle::State & previous_state)
{
  logTransition("on_activate", previous_state);

  // 每次激活都重置：同一对象可能被 deactivate/activate 多次
  started_ = false;
  read_count_ = 0;
  write_count_ = 0;

  pos_ = 0.0;        // ★ 新增：保证每次激活从 0 开始，实验可重复
  target_ = 0.0;     // ★ 新增


  return CallbackReturn::SUCCESS;
}

CallbackReturn SineJoint::on_deactivate(const rclcpp_lifecycle::State & previous_state)
{
  logTransition("on_deactivate", previous_state);
  RCLCPP_INFO(
    rclcpp::get_logger("SineJoint"),
    "            累计调用: read=%lu 次, write=%lu 次", read_count_, write_count_);
  return CallbackReturn::SUCCESS;
}

CallbackReturn SineJoint::on_cleanup(const rclcpp_lifecycle::State & previous_state)
{
  logTransition("on_cleanup", previous_state);
  return CallbackReturn::SUCCESS;
}

CallbackReturn SineJoint::on_shutdown(const rclcpp_lifecycle::State & previous_state)
{
  logTransition("on_shutdown", previous_state);
  return CallbackReturn::SUCCESS;
}

CallbackReturn SineJoint::on_error(const rclcpp_lifecycle::State & previous_state)
{
  RCLCPP_ERROR(
    rclcpp::get_logger("SineJoint"),
    "[LIFECYCLE] %-14s previous_state=%s", "on_error", previous_state.label().c_str());
  return CallbackReturn::SUCCESS;
}

// ─────────────────────────────────────────────────────────────
// 导出接口：只把变量地址交出去，之后 controller_manager 直接读写这块内存
// ─────────────────────────────────────────────────────────────
std::vector<hardware_interface::StateInterface>
SineJoint::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> si;
  si.emplace_back(
    info_.joints[0].name,
    hardware_interface::HW_IF_POSITION,
    &pos_);

  RCLCPP_INFO(
    rclcpp::get_logger("SineJoint"),
    "[EXPORT] state   : %s/%s -> &pos_",
    info_.joints[0].name.c_str(), hardware_interface::HW_IF_POSITION);

  return si;
}

std::vector<hardware_interface::CommandInterface>
SineJoint::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> ci;
  ci.emplace_back(
    info_.joints[0].name,
    hardware_interface::HW_IF_POSITION,
    &cmd_pos_);

  RCLCPP_INFO(
    rclcpp::get_logger("SineJoint"),
    "[EXPORT] command : %s/%s -> &cmd_pos_",
    info_.joints[0].name.c_str(), hardware_interface::HW_IF_POSITION);

  return ci;
}

// ─────────────────────────────────────────────────────────────
// 周期回调
//   read  : 硬件真实值 -> pos_      （只有 active 状态才会被调用）
//   write : cmd_pos_   -> 硬件
// ─────────────────────────────────────────────────────────────
return_type SineJoint::read(const rclcpp::Time &, const rclcpp::Duration &period)
{
  ++read_count_;

  const double dt = period.seconds();        // ★ 用真实步长，不写死 0.01


  // 一阶惯性：用解析解，dt 变大也不会发散
  //   pos_{k+1} = target + (pos_k - target) * exp(-dt/tau)
  const double alpha = 1.0 - std::exp(-dt / std::max(tau_, 1e-6));
  const double desired = pos_ + (target_ - pos_) * alpha;

  // 速度限位：本周期最多变化 max_vel * dt
  const double max_step = max_vel_ * dt;
  const double step = std::clamp(desired - pos_, -max_step, max_step);

  // 位置限位
  pos_ = std::clamp(pos_ + step, -max_pos_, max_pos_);

  if (read_count_ % 100 == 1) {
    RCLCPP_INFO(
      rclcpp::get_logger("SineJoint"),
      "[PERIOD] read  #%lu  pos=%.4f  target=%.4f  gap=%.4f",
      read_count_, pos_, target_, target_ - pos_);
  }

  return return_type::OK;
}

return_type SineJoint::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  ++write_count_;


  // 目标限幅：超过机械行程就不该下发
  target_ = std::clamp(cmd_pos_, -max_pos_, max_pos_);

  if (write_count_ % 100 == 1) {
    RCLCPP_INFO(
      rclcpp::get_logger("SineJoint"),
      "[PERIOD] write #%lu  cmd=%.4f -> target=%.4f", write_count_, cmd_pos_, target_);
  }
  return return_type::OK;
}

}  // namespace my_hardware

// 注册插件：第一个参数是 C++ 类，第二个是它继承的基类
PLUGINLIB_EXPORT_CLASS(my_hardware::SineJoint, hardware_interface::SystemInterface)