#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include "mini_slam/pose2d.hpp"
#include "mini_slam/ekf.hpp"

namespace mini_slam
{

class EkfNode : public rclcpp::Node
{
public:
  EkfNode() : Node("ekf_mini")
  {
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/odom");
    imu_topic_ = declare_parameter<std::string>("imu_topic", "/imu");
    out_topic_ = declare_parameter<std::string>("output_topic", "/odometry/filtered");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    publish_tf_ = declare_parameter<bool>("publish_tf", true);
    const double freq = declare_parameter<double>("frequency", 30.0);

    sigma_v_ = declare_parameter<double>("sigma_v", 0.05);
    sigma_w_ = declare_parameter<double>("sigma_w", 0.05);
    sigma_yaw_rate_ = declare_parameter<double>("sigma_yaw_rate", 0.02);

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, rclcpp::SensorDataQoS(),
      [this](nav_msgs::msg::Odometry::SharedPtr m) {onOdom(m);});

    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      imu_topic_, rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::Imu::SharedPtr m) {onImu(m);});

    pub_ = create_publisher<nav_msgs::msg::Odometry>(out_topic_, 10);

    if (publish_tf_) {
      tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    }

    last_time_ = now();
    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / freq),
      [this]() {tick();});

    RCLCPP_INFO(get_logger(),
      "ekf_mini 启动：%s + %s -> %s，%.1f Hz，tf=%s",
      odom_topic_.c_str(), imu_topic_.c_str(), out_topic_.c_str(), freq,
      publish_tf_ ? "on" : "off");
  }

private:
  void onOdom(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    latest_odom_ = msg;
    has_odom_ = true;
  }

  void onImu(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    latest_imu_ = msg;
    has_imu_ = true;
  }

  void tick()
  {
    const rclcpp::Time t = now();
    const double dt = (t - last_time_).seconds();
    if (dt <= 0.0 || dt > 1.0) {last_time_ = t; return;}

    // 第一次收到 odom 时初始化到它的位姿
    if (!ekf_.initialized()) {
      if (!has_odom_) {last_time_ = t; return;}
      const auto & p = latest_odom_->pose.pose;
      ekf_.initialize(p.position.x, p.position.y, yawFromQuat(p.orientation));
      last_time_ = t;
      RCLCPP_INFO(get_logger(), "EKF 初始化为 (%.3f, %.3f, %.3f)",
        p.position.x, p.position.y, yawFromQuat(p.orientation));
      return;
    }

    ekf_.predict(dt, sigma_v_, sigma_w_);

    if (has_odom_) {
      const auto & tw = latest_odom_->twist.twist;
      ekf_.updateOdomTwist(tw.linear.x, tw.angular.z, sigma_v_, sigma_w_);
    }
    if (has_imu_) {
      ekf_.updateYawRate(latest_imu_->angular_velocity.z, sigma_yaw_rate_);
    }

    publish(t);
    last_time_ = t;
  }

  void publish(const rclcpp::Time & stamp)
  {
    nav_msgs::msg::Odometry out;
    out.header.stamp = stamp;
    out.header.frame_id = odom_frame_;
    out.child_frame_id = base_frame_;
    out.pose.pose.position.x = ekf_.x();
    out.pose.pose.position.y = ekf_.y();
    out.pose.pose.position.z = 0.0;
    out.pose.pose.orientation = quatFromYaw(ekf_.yaw());
    out.twist.twist.linear.x = ekf_.vx();
    out.twist.twist.angular.z = ekf_.vyaw();
    pub_->publish(out);

    if (tf_broadcaster_) {
      geometry_msgs::msg::TransformStamped tf;
      tf.header.stamp = stamp;
      tf.header.frame_id = odom_frame_;
      tf.child_frame_id = base_frame_;
      tf.transform.translation.x = ekf_.x();
      tf.transform.translation.y = ekf_.y();
      tf.transform.rotation = quatFromYaw(ekf_.yaw());
      tf_broadcaster_->sendTransform(tf);
    }
  }

  static double yawFromQuat(const geometry_msgs::msg::Quaternion & q)
  {
    return std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  }

  static geometry_msgs::msg::Quaternion quatFromYaw(double yaw)
  {
    geometry_msgs::msg::Quaternion q;
    q.x = 0.0; q.y = 0.0;
    q.z = std::sin(yaw * 0.5);
    q.w = std::cos(yaw * 0.5);
    return q;
  }

  std::string odom_topic_, imu_topic_, out_topic_, odom_frame_, base_frame_;
  bool publish_tf_{true};
  double sigma_v_{0.05}, sigma_w_{0.05}, sigma_yaw_rate_{0.02};

  Ekf2D ekf_;
  nav_msgs::msg::Odometry::SharedPtr latest_odom_;
  sensor_msgs::msg::Imu::SharedPtr latest_imu_;
  bool has_odom_{false}, has_imu_{false};
  rclcpp::Time last_time_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace mini_slam

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<mini_slam::EkfNode>());
  rclcpp::shutdown();
  return 0;
}
