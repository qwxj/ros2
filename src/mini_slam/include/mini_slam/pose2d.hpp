#ifndef MINI_SLAM__POSE2D_HPP_
#define MINI_SLAM__POSE2D_HPP_

#include <cmath>

namespace mini_slam
{

struct Pose2D
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

/// 把角度归一化到 [-pi, pi]
inline double wrapToPi(double a)
{
  while (a > M_PI) {a -= 2.0 * M_PI;}
  while (a < -M_PI) {a += 2.0 * M_PI;}
  return a;
}

}  // namespace mini_slam

#endif