// L0 练习：一个真正的"纯 C++ 库"，不 include 任何 rclcpp 头文件。
// 这样它既能被 ROS 节点用，也能被 gtest 直接测（L4/L6 会用上）。
#ifndef ROBOT_MATH__DIFF_DRIVE_HPP_
#define ROBOT_MATH__DIFF_DRIVE_HPP_

namespace robot_math
{

struct WheelSpeed
{
  double left_rad_s;
  double right_rad_s;
};

struct BodyTwist
{
  double linear_x;   // m/s
  double angular_z;  // rad/s
};

/// 两轮差速运动学：机体速度 <-> 轮速，以及一次控制周期的里程计积分。
class DiffDriveKinematics
{
public:
  /// 参数非法（<= 0）时抛 std::invalid_argument。
  DiffDriveKinematics(double wheel_radius, double wheel_separation);

  /// v, w -> 左右轮角速度
  WheelSpeed bodyToWheel(const BodyTwist & twist) const;

  /// 左右轮角速度 -> v, w
  BodyTwist wheelToBody(const WheelSpeed & wheel) const;

  /// 中点法积分：把一次控制周期的轮速累加到位姿 (x, y, yaw)。
  void integrate(
    double dt, const WheelSpeed & wheel,
    double & x, double & y, double & yaw) const;

  double wheelRadius() const {return wheel_radius_;}
  double wheelSeparation() const {return wheel_separation_;}

private:
  double wheel_radius_;
  double wheel_separation_;
};

}  // namespace robot_math

#endif  // ROBOT_MATH__DIFF_DRIVE_HPP_
