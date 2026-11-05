// L0 练习节点：订阅 /cmd_vel，用 robot_math 换算成左右轮角速度，发布 /wheel_speed。
// 它存在的意义不是"功能"，而是证明 robot_math 的 find_package 导出链路是通的。
#include <memory>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robot_math/diff_drive.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

class WheelSpeedNode : public rclcpp::Node
{
public:
  WheelSpeedNode()
  : rclcpp::Node("wheel_speed_node"),
    kinematics_(
      this->declare_parameter<double>("wheel_radius", 0.0325),
      this->declare_parameter<double>("wheel_separation", 0.205))
  {
    wheel_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("wheel_speed", 10);

    cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10,
      [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
        const robot_math::WheelSpeed wheel =
          kinematics_.bodyToWheel({msg->linear.x, msg->angular.z});

        std_msgs::msg::Float64MultiArray out;
        out.data = {wheel.left_rad_s, wheel.right_rad_s};
        wheel_pub_->publish(out);

        RCLCPP_INFO_THROTTLE(
          this->get_logger(), *this->get_clock(), 1000,
          "vx=%.3f m/s  wz=%.3f rad/s  ->  L=%.2f  R=%.2f rad/s",
          msg->linear.x, msg->angular.z, wheel.left_rad_s, wheel.right_rad_s);
      });

    RCLCPP_INFO(
      this->get_logger(),
      "wheel_radius=%.4f m, wheel_separation=%.4f m, 10 rad/s 对应 v=%.3f m/s",
      kinematics_.wheelRadius(), kinematics_.wheelSeparation(),
      kinematics_.wheelRadius() * 10.0);
  }

private:
  robot_math::DiffDriveKinematics kinematics_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr wheel_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WheelSpeedNode>());
  rclcpp::shutdown();
  return 0;
}
