#ifndef MY_ROBOT_CONTROL__IMU_SENSOR_HPP_
#define MY_ROBOT_CONTROL__IMU_SENSOR_HPP_

#include <random>
#include <string>
#include <vector>

#include "hardware_interface/sensor_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace my_robot_control
{

// IMU 传感器硬件接口（ros2_control 的第三种硬件类型：只读，没有 command interface）。
// 它从进程内的 ground_truth() 取真实运动，叠加偏置和白噪声，冒充真实 IMU。
class ImuSensor : public hardware_interface::SensorInterface
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

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  std::string sensor_name_{"imu_sensor"};

  std::vector<double> orientation_{0.0, 0.0, 0.0, 1.0};   // x y z w
  std::vector<double> angular_velocity_{0.0, 0.0, 0.0};   // rad/s
  std::vector<double> linear_acceleration_{0.0, 0.0, 9.81};

  double noise_stddev_{0.02};
  double bias_{0.005};
  double wheel_radius_{0.05};
  std::mt19937 rng_;
  bool configured_{false};
};

}  // namespace my_robot_control

#endif  // MY_ROBOT_CONTROL__IMU_SENSOR_HPP_
