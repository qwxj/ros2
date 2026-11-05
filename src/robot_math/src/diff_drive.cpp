#include "robot_math/diff_drive.hpp"

#include <cmath>
#include <stdexcept>

namespace robot_math
{
namespace
{
constexpr double kPi = 3.14159265358979323846;

void wrapToPi(double & angle)
{
  while (angle > kPi) {
    angle -= 2.0 * kPi;
  }
  while (angle < -kPi) {
    angle += 2.0 * kPi;
  }
}
}  // namespace

DiffDriveKinematics::DiffDriveKinematics(double wheel_radius, double wheel_separation)
: wheel_radius_(wheel_radius), wheel_separation_(wheel_separation)
{
  if (!(wheel_radius > 0.0)) {
    throw std::invalid_argument("wheel_radius must be > 0");
  }
  if (!(wheel_separation > 0.0)) {
    throw std::invalid_argument("wheel_separation must be > 0");
  }
}

WheelSpeed DiffDriveKinematics::bodyToWheel(const BodyTwist & twist) const
{
  const double half_track = wheel_separation_ * 0.5;
  return WheelSpeed{
    (twist.linear_x - twist.angular_z * half_track) / wheel_radius_,
    (twist.linear_x + twist.angular_z * half_track) / wheel_radius_};
}

BodyTwist DiffDriveKinematics::wheelToBody(const WheelSpeed & wheel) const
{
  const double v_left = wheel.left_rad_s * wheel_radius_;
  const double v_right = wheel.right_rad_s * wheel_radius_;
  return BodyTwist{
    0.5 * (v_left + v_right),
    (v_right - v_left) / wheel_separation_};
}

void DiffDriveKinematics::integrate(
  double dt, const WheelSpeed & wheel,
  double & x, double & y, double & yaw) const
{
  const BodyTwist twist = wheelToBody(wheel);
  const double delta_yaw = twist.angular_z * dt;
  const double mid_yaw = yaw + 0.5 * delta_yaw;
  x += twist.linear_x * std::cos(mid_yaw) * dt;
  y += twist.linear_x * std::sin(mid_yaw) * dt;
  yaw += delta_yaw;
  wrapToPi(yaw);
}

}  // namespace robot_math
