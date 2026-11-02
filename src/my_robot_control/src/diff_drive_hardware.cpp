#include "my_robot_control/diff_drive_hardware.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <string>

#include "my_robot_control/ground_truth.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace my_robot_control
{

namespace
{
double clampd(double v, double lo, double hi)
{
  return std::max(lo, std::min(hi, v));
}

double param_or(const hardware_interface::HardwareInfo & info, const std::string & key, double def)
{
  auto it = info.hardware_parameters.find(key);
  if (it == info.hardware_parameters.end()) {return def;}
  try {return std::stod(it->second);} catch (...) {return def;}
}

std::string param_str(
  const hardware_interface::HardwareInfo & info, const std::string & key, const std::string & def)
{
  auto it = info.hardware_parameters.find(key);
  return (it == info.hardware_parameters.end()) ? def : it->second;
}
}  // namespace


// ─────────────────────────────────────────────────────────────
// 1. on_init：只调一次。读 <ros2_control> 的 <param>，校验 joint 数量
// ─────────────────────────────────────────────────────────────
hardware_interface::CallbackReturn
DiffDriveHardware::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (info_.joints.size() != 2) {
    RCLCPP_ERROR(
      rclcpp::get_logger("DiffDriveHardware"), "只支持 2 个轮子，实际 %zu", info_.joints.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  motor_mode_ = param_str(info_, "motor_mode", "velocity");
  motor_inertia_ = param_or(info_, "motor_inertia", 0.5);
  viscous_friction_ = param_or(info_, "viscous_friction", 0.05);
  vel_gain_ = param_or(info_, "vel_gain", 2.0);
  max_effort_ = param_or(info_, "max_effort", 5.0);
  max_wheel_speed_ = param_or(info_, "max_wheel_speed", 10.0);
  encoder_cpr_ = param_or(info_, "encoder_cpr", 4096.0);
  wheel_separation_ = param_or(info_, "wheel_separation", 0.30);
  wheel_radius_ = param_or(info_, "wheel_radius", 0.05);

  RCLCPP_INFO(
    rclcpp::get_logger("DiffDriveHardware"),
    "on_init: motor_mode=%s  J=%.3f  b=%.3f  Kp=%.2f  max_effort=%.2f  CPR=%.0f  L=%.3f  r=%.3f",
    motor_mode_.c_str(), motor_inertia_, viscous_friction_, vel_gain_, max_effort_,
    encoder_cpr_, wheel_separation_, wheel_radius_);

  // ★★★ 必须在 on_init 里分配 ★★★
  // 因为 export_state_interfaces() / export_command_interfaces() 是在
  // on_init 阶段被 resource_manager 调用的，框架会永久记住这些成员的地址。
  // 如果等到 on_configure 才 assign/resize，vector 会重新分配内存，
  // 框架手里的指针立刻变成悬空指针 —— 运行时表现为
  //   ReadOnlyHandle::get_value() failed. value_ptr_ is null.
  // 然后 ros2_control_node 直接 SIGABRT 崩掉。
  const std::size_t n = info_.joints.size();
  position_.assign(n, 0.0);
  position_raw_.assign(n, 0.0);
  velocity_.assign(n, 0.0);
  effort_.assign(n, 0.0);
  velocity_command_.assign(n, 0.0);
  effort_command_.assign(n, 0.0);

  return hardware_interface::CallbackReturn::SUCCESS;
}


// ─────────────────────────────────────────────────────────────
// 2. on_configure：可反复调。这里【只清零，绝不重新分配】
//    （重新分配会让 on_init 导出的指针悬空，直接 SIGABRT）
// ─────────────────────────────────────────────────────────────
hardware_interface::CallbackReturn
DiffDriveHardware::on_configure(const rclcpp_lifecycle::State & previous_state)
{
  (void)previous_state;
  std::fill(position_.begin(), position_.end(), 0.0);
  std::fill(position_raw_.begin(), position_raw_.end(), 0.0);
  std::fill(velocity_.begin(), velocity_.end(), 0.0);
  std::fill(effort_.begin(), effort_.end(), 0.0);
  std::fill(velocity_command_.begin(), velocity_command_.end(), 0.0);
  std::fill(effort_command_.begin(), effort_command_.end(), 0.0);

  configured_ = true;
  RCLCPP_INFO(
    rclcpp::get_logger("DiffDriveHardware"), "on_configure: 已就绪 (%zu 关节)", position_.size());
  return hardware_interface::CallbackReturn::SUCCESS;
}


// ─────────────────────────────────────────────────────────────
// 3. export_state_interfaces：告诉框架"我能提供哪些状态"
//    返回的是【指针】，控制器直接通过它读写这些成员变量
// ─────────────────────────────────────────────────────────────
std::vector<hardware_interface::StateInterface>
DiffDriveHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> out;
  for (std::size_t i = 0; i < info_.joints.size(); ++i) {
    const auto & name = info_.joints[i].name;
    out.emplace_back(name, hardware_interface::HW_IF_POSITION, &position_[i]);
    out.emplace_back(name, hardware_interface::HW_IF_VELOCITY, &velocity_[i]);
    out.emplace_back(name, hardware_interface::HW_IF_EFFORT, &effort_[i]);   // 电流/力矩反馈
  }
  return out;
}


// ─────────────────────────────────────────────────────────────
// 4. export_command_interfaces：告诉框架"我能接受哪些指令"
//    velocity 和 effort 两种都暴露，由 motor_mode 参数决定 write() 用哪个
// ─────────────────────────────────────────────────────────────
std::vector<hardware_interface::CommandInterface>
DiffDriveHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> out;
  for (std::size_t i = 0; i < info_.joints.size(); ++i) {
    const auto & name = info_.joints[i].name;
    out.emplace_back(name, hardware_interface::HW_IF_VELOCITY, &velocity_command_[i]);
    out.emplace_back(name, hardware_interface::HW_IF_EFFORT, &effort_command_[i]);
  }
  return out;
}


// ─────────────────────────────────────────────────────────────
// 5. on_activate：上电
// ─────────────────────────────────────────────────────────────
hardware_interface::CallbackReturn
DiffDriveHardware::on_activate(const rclcpp_lifecycle::State & previous_state)
{
  (void)previous_state;
  std::fill(position_.begin(), position_.end(), 0.0);
  std::fill(position_raw_.begin(), position_raw_.end(), 0.0);
  std::fill(velocity_.begin(), velocity_.end(), 0.0);
  std::fill(effort_.begin(), effort_.end(), 0.0);
  std::fill(velocity_command_.begin(), velocity_command_.end(), 0.0);
  std::fill(effort_command_.begin(), effort_command_.end(), 0.0);

  ground_truth() = GroundTruth{};
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveHardware"), "on_activate: 已上电，状态清零");
  return hardware_interface::CallbackReturn::SUCCESS;
}


// ─────────────────────────────────────────────────────────────
// 6. on_deactivate：下电（不清位置，允许恢复）
// ─────────────────────────────────────────────────────────────
hardware_interface::CallbackReturn
DiffDriveHardware::on_deactivate(const rclcpp_lifecycle::State & previous_state)
{
  (void)previous_state;
  std::fill(velocity_.begin(), velocity_.end(), 0.0);
  std::fill(velocity_command_.begin(), velocity_command_.end(), 0.0);
  std::fill(effort_command_.begin(), effort_command_.end(), 0.0);
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveHardware"), "on_deactivate: 已下电");
  return hardware_interface::CallbackReturn::SUCCESS;
}


// ─────────────────────────────────────────────────────────────
// 7. on_cleanup：关设备。只能清零，【不能 clear()】
//    （clear() 会释放内存，on_init 导出的指针同样会悬空）
// ─────────────────────────────────────────────────────────────
hardware_interface::CallbackReturn
DiffDriveHardware::on_cleanup(const rclcpp_lifecycle::State & previous_state)
{
  (void)previous_state;
  std::fill(position_.begin(), position_.end(), 0.0);
  std::fill(position_raw_.begin(), position_raw_.end(), 0.0);
  std::fill(velocity_.begin(), velocity_.end(), 0.0);
  std::fill(effort_.begin(), effort_.end(), 0.0);
  std::fill(velocity_command_.begin(), velocity_command_.end(), 0.0);
  std::fill(effort_command_.begin(), effort_command_.end(), 0.0);
  configured_ = false;
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveHardware"), "on_cleanup: 已复位（内存保留）");
  return hardware_interface::CallbackReturn::SUCCESS;
}


// ─────────────────────────────────────────────────────────────
// 8. on_error：出错复位
// ─────────────────────────────────────────────────────────────
hardware_interface::CallbackReturn
DiffDriveHardware::on_error(const rclcpp_lifecycle::State & previous_state)
{
  (void)previous_state;
  RCLCPP_ERROR(rclcpp::get_logger("DiffDriveHardware"), "on_error: 进入错误处理，强制停车");
  std::fill(velocity_.begin(), velocity_.end(), 0.0);
  std::fill(velocity_command_.begin(), velocity_command_.end(), 0.0);
  std::fill(effort_command_.begin(), effort_command_.end(), 0.0);
  return hardware_interface::CallbackReturn::SUCCESS;
}


// ─────────────────────────────────────────────────────────────
// 9. read：硬件 -> 框架（模拟编码器）
//    位置积分 + 4096 线量化（真实编码器只能读到离散刻度）
// ─────────────────────────────────────────────────────────────
hardware_interface::return_type
DiffDriveHardware::read(const rclcpp::Time & time, const rclcpp::Duration & period)
{
  (void)time;
  if (!configured_) {return hardware_interface::return_type::ERROR;}

  const double dt = period.seconds();
  const double res = 2.0 * M_PI / encoder_cpr_;     // 一个刻度的弧度

  for (std::size_t i = 0; i < position_.size(); ++i) {
    position_raw_[i] += velocity_[i] * dt;
    // 量化：真实编码器只能给出 res 的整数倍
    position_[i] = std::round(position_raw_[i] / res) * res;
  }

  // 写入"真实世界"，供 ImuSensor 读取
  auto & gt = ground_truth();
  if (position_.size() == 2) {
    gt.left_vel = velocity_[0];
    gt.right_vel = velocity_[1];
    gt.yaw_rate = (velocity_[1] - velocity_[0]) * wheel_radius_ / wheel_separation_;
    gt.yaw += gt.yaw_rate * dt;
  }

  return hardware_interface::return_type::OK;
}


// ─────────────────────────────────────────────────────────────
// 10. write：框架 -> 硬件（模拟电机）
//     物理模型：力矩 -> 角加速度 -> 角速度（含惯性和粘滞摩擦）
//       dω/dt = (τ - b·ω) / J
//     velocity 模式：τ = b·ω_cmd + Kp·(ω_cmd - ω)  -> 稳态 ω = ω_cmd，但有动态滞后
//     effort   模式：τ = τ_cmd                     -> 纯力矩驱动
// ─────────────────────────────────────────────────────────────
hardware_interface::return_type
DiffDriveHardware::write(const rclcpp::Time & time, const rclcpp::Duration & period)
{
  (void)time;
  if (!configured_) {return hardware_interface::return_type::ERROR;}

  const double dt = period.seconds();

  for (std::size_t i = 0; i < velocity_.size(); ++i) {
    double torque;

    if (motor_mode_ == "effort") {
      torque = clampd(effort_command_[i], -max_effort_, max_effort_);
    } else {
      const double target = clampd(velocity_command_[i], -max_wheel_speed_, max_wheel_speed_);
      torque = viscous_friction_ * target + vel_gain_ * (target - velocity_[i]);
      torque = clampd(torque, -max_effort_, max_effort_);
    }

    const double accel = (torque - viscous_friction_ * velocity_[i]) / motor_inertia_;
    velocity_[i] = clampd(velocity_[i] + accel * dt, -max_wheel_speed_, max_wheel_speed_);
    effort_[i] = torque;      // 力矩反馈（真实硬件是电流采样）
  }

  return hardware_interface::return_type::OK;
}

}  // namespace my_robot_control


PLUGINLIB_EXPORT_CLASS(
  my_robot_control::DiffDriveHardware,
  hardware_interface::SystemInterface)
