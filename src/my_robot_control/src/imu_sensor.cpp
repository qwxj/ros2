#include "my_robot_control/imu_sensor.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <random>
#include <string>

#include "my_robot_control/ground_truth.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace my_robot_control
{

namespace
{
double param_or(const hardware_interface::HardwareInfo & info, const std::string & key, double def)
{
  auto it = info.hardware_parameters.find(key);
  if (it == info.hardware_parameters.end()) {return def;}
  try {return std::stod(it->second);} catch (...) {return def;}
}
}  // namespace


hardware_interface::CallbackReturn
ImuSensor::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SensorInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (info_.sensors.size() != 1) {
    RCLCPP_ERROR(
      rclcpp::get_logger("ImuSensor"), "只支持 1 个 IMU，实际 %zu", info_.sensors.size());
    return hardware_interface::CallbackReturn::ERROR;
  }
  sensor_name_ = info_.sensors[0].name;

  noise_stddev_ = param_or(info_, "noise_stddev", 0.02);    // rad/s 白噪声
  bias_ = param_or(info_, "bias", 0.005);                   // rad/s 常值偏置
  wheel_radius_ = param_or(info_, "wheel_radius", 0.05);
  rng_.seed(static_cast<unsigned>(param_or(info_, "seed", 42.0)));

  RCLCPP_INFO(
    rclcpp::get_logger("ImuSensor"),
    "on_init: sensor=%s  noise=%.4f  bias=%.4f", sensor_name_.c_str(), noise_stddev_, bias_);
  return hardware_interface::CallbackReturn::SUCCESS;
}


hardware_interface::CallbackReturn
ImuSensor::on_configure(const rclcpp_lifecycle::State & previous_state)
{
  (void)previous_state;
  // 注意：只能用 fill，不能用 orientation_ = {...} 这种赋值
  // （赋值在某些实现下会重新分配 vector，让 export 出去的指针悬空）
  std::fill(orientation_.begin(), orientation_.end(), 0.0);
  orientation_[3] = 1.0;                          // w = 1
  std::fill(angular_velocity_.begin(), angular_velocity_.end(), 0.0);
  std::fill(linear_acceleration_.begin(), linear_acceleration_.end(), 0.0);
  linear_acceleration_[2] = 9.81;                 // 静止时 z 轴 1g
  configured_ = true;
  RCLCPP_INFO(rclcpp::get_logger("ImuSensor"), "on_configure: 就绪");
  return hardware_interface::CallbackReturn::SUCCESS;
}


hardware_interface::CallbackReturn
ImuSensor::on_activate(const rclcpp_lifecycle::State & previous_state)
{
  (void)previous_state;
  RCLCPP_INFO(rclcpp::get_logger("ImuSensor"), "on_activate");
  return hardware_interface::CallbackReturn::SUCCESS;
}


hardware_interface::CallbackReturn
ImuSensor::on_deactivate(const rclcpp_lifecycle::State & previous_state)
{
  (void)previous_state;
  std::fill(angular_velocity_.begin(), angular_velocity_.end(), 0.0);
  RCLCPP_INFO(rclcpp::get_logger("ImuSensor"), "on_deactivate");
  return hardware_interface::CallbackReturn::SUCCESS;
}


hardware_interface::CallbackReturn
ImuSensor::on_cleanup(const rclcpp_lifecycle::State & previous_state)
{
  (void)previous_state;
  configured_ = false;
  RCLCPP_INFO(rclcpp::get_logger("ImuSensor"), "on_cleanup");
  return hardware_interface::CallbackReturn::SUCCESS;
}


hardware_interface::CallbackReturn
ImuSensor::on_error(const rclcpp_lifecycle::State & previous_state)
{
  (void)previous_state;
  RCLCPP_ERROR(rclcpp::get_logger("ImuSensor"), "on_error");
  std::fill(angular_velocity_.begin(), angular_velocity_.end(), 0.0);
  return hardware_interface::CallbackReturn::SUCCESS;
}


std::vector<hardware_interface::StateInterface>
ImuSensor::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> out;
  out.emplace_back(sensor_name_, "orientation.x", &orientation_[0]);
  out.emplace_back(sensor_name_, "orientation.y", &orientation_[1]);
  out.emplace_back(sensor_name_, "orientation.z", &orientation_[2]);
  out.emplace_back(sensor_name_, "orientation.w", &orientation_[3]);
  out.emplace_back(sensor_name_, "angular_velocity.x", &angular_velocity_[0]);
  out.emplace_back(sensor_name_, "angular_velocity.y", &angular_velocity_[1]);
  out.emplace_back(sensor_name_, "angular_velocity.z", &angular_velocity_[2]);
  out.emplace_back(sensor_name_, "linear_acceleration.x", &linear_acceleration_[0]);
  out.emplace_back(sensor_name_, "linear_acceleration.y", &linear_acceleration_[1]);
  out.emplace_back(sensor_name_, "linear_acceleration.z", &linear_acceleration_[2]);
  return out;
}


// read：从"真实世界"采样，叠加偏置和白噪声
hardware_interface::return_type
ImuSensor::read(const rclcpp::Time & time, const rclcpp::Duration & period)
{
  (void)time;
  (void)period;
  if (!configured_) {return hardware_interface::return_type::ERROR;}

  const auto & gt = ground_truth();

  std::normal_distribution<double> noise(0.0, noise_stddev_);

  const double yaw = gt.yaw;
  const double yaw_rate = gt.yaw_rate + bias_ + noise(rng_);

  // 欧拉角 -> 四元数（只有 yaw 非零）
  const double half = yaw * 0.5;
  orientation_[0] = 0.0;
  orientation_[1] = 0.0;
  orientation_[2] = std::sin(half);
  orientation_[3] = std::cos(half);

  angular_velocity_[0] = 0.0;
  angular_velocity_[1] = 0.0;
  angular_velocity_[2] = yaw_rate;

  // 差速小车的向心加速度投影到车体 y 轴
  const double v = 0.5 * (gt.left_vel + gt.right_vel) * wheel_radius_;
  linear_acceleration_[0] = 0.0;
  linear_acceleration_[1] = v * yaw_rate;
  linear_acceleration_[2] = 9.81;

  return hardware_interface::return_type::OK;
}

}  // namespace my_robot_control


PLUGINLIB_EXPORT_CLASS(
  my_robot_control::ImuSensor,
  hardware_interface::SensorInterface)
