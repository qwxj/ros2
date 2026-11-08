#include "mini_slam/scan_matcher.hpp"

#include "mini_slam/distance_field.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace mini_slam
{

void ScanMatcher::setMap(const OccupancyGrid & map)
{
  map_w_ = map.width();
  map_h_ = map.height();
  map_res_ = map.resolution();
  map_origin_x_ = map.originX();
  map_origin_y_ = map.originY();

  // O(W*H) 的 chamfer 距离变换（暴力版在 400x400 图上要几百毫秒）
  dist_field_ = buildDistanceField(map);
  has_map_ = true;
}

double ScanMatcher::rawScore(double wx, double wy, double yaw,
                             const std::vector<float> & ranges,
                             double angle_min, double angle_increment,
                             double range_max,
                             double sigma) const
{
  const double inv_2sigma2 = 1.0 / (2.0 * sigma * sigma);
  double sum = 0.0;
  int used = 0;

  for (size_t i = 0; i < ranges.size(); ++i) {
    const double r = ranges[i];
    if (!std::isfinite(r) || r < 0.05 || r >= range_max) {continue;}

    const double a = yaw + angle_min + static_cast<double>(i) * angle_increment;
    const double ex = wx + r * std::cos(a);
    const double ey = wy + r * std::sin(a);

    const int gx = static_cast<int>(std::floor((ex - map_origin_x_) / map_res_));
    const int gy = static_cast<int>(std::floor((ey - map_origin_y_) / map_res_));
    if (gx < 0 || gx >= map_w_ || gy < 0 || gy >= map_h_) {continue;}

    const double d = dist_field_[static_cast<size_t>(gy) * map_w_ + gx];
    sum += std::exp(-d * d * inv_2sigma2);
    ++used;
  }

  if (used == 0) {return 0.0;}
  return sum / static_cast<double>(used);
}

double ScanMatcher::scoreAt(double x, double y, double yaw,
                            const std::vector<float> & ranges,
                            double angle_min, double angle_increment,
                            double range_max,
                            double sigma) const
{
  if (!has_map_) {return 0.0;}
  return rawScore(x, y, yaw, ranges, angle_min, angle_increment, range_max, sigma);
}

ScanMatchResult ScanMatcher::match(const std::vector<float> & ranges,
                                   double angle_min, double angle_increment,
                                   double range_max,
                                   double guess_x, double guess_y, double guess_yaw,
                                   double search_xy, double search_yaw,
                                   int steps, double sigma) const
{
  ScanMatchResult res;
  res.x = guess_x;
  res.y = guess_y;
  res.yaw = guess_yaw;
  if (!has_map_ || steps < 2) {return res;}

  double best_score = -1.0;
  double cx = guess_x, cy = guess_y, cyaw = guess_yaw;

  // ---------- 阶段 1：粗搜（范围大、步长大）----------
  for (int iy = 0; iy < steps; ++iy) {
    const double dy = -search_xy + 2.0 * search_xy * iy / (steps - 1);
    for (int ix = 0; ix < steps; ++ix) {
      const double dx = -search_xy + 2.0 * search_xy * ix / (steps - 1);
      for (int ia = 0; ia < steps; ++ia) {
        const double da = -search_yaw + 2.0 * search_yaw * ia / (steps - 1);
        const double s = rawScore(guess_x + dx, guess_y + dy, guess_yaw + da,
                                  ranges, angle_min, angle_increment, range_max, sigma);
        ++res.candidates;
        if (s > best_score) {
          best_score = s;
          cx = guess_x + dx;
          cy = guess_y + dy;
          cyaw = guess_yaw + da;
        }
      }
    }
  }

  // ---------- 阶段 2：细搜（在粗搜最优点附近，范围 = 一个粗步长）----------
  const double step_xy = 2.0 * search_xy / (steps - 1);
  const double step_yaw = 2.0 * search_yaw / (steps - 1);

  for (int iy = 0; iy < steps; ++iy) {
    const double dy = -step_xy + 2.0 * step_xy * iy / (steps - 1);
    for (int ix = 0; ix < steps; ++ix) {
      const double dx = -step_xy + 2.0 * step_xy * ix / (steps - 1);
      for (int ia = 0; ia < steps; ++ia) {
        const double da = -step_yaw + 2.0 * step_yaw * ia / (steps - 1);
        const double s = rawScore(cx + dx, cy + dy, cyaw + da,
                                  ranges, angle_min, angle_increment, range_max, sigma);
        ++res.candidates;
        if (s > best_score) {
          best_score = s;
          res.x = cx + dx;
          res.y = cy + dy;
          res.yaw = cyaw + da;
        }
      }
    }
  }

  if (best_score < 0.0) {
    res.x = cx;
    res.y = cy;
    res.yaw = cyaw;
    best_score = 0.0;
  }
  res.score = best_score;
  return res;
}

}  // namespace mini_slam