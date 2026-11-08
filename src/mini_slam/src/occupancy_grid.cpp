#include "mini_slam/occupancy_grid.hpp"

#include <algorithm>
#include <cmath>

namespace mini_slam
{

OccupancyGrid::OccupancyGrid(double resolution, double origin_x, double origin_y,
                             int width, int height)
: resolution_(resolution), origin_x_(origin_x), origin_y_(origin_y),
  width_(width), height_(height),
  log_odds_(static_cast<size_t>(width) * height, 0.0),
  observed_(static_cast<size_t>(width) * height, 0)
{
}

// ------------------------------------------------------------------
// 坐标转换
//   栅格 (0,0) 的中心 = origin + (0.5, 0.5) * resolution
//   floor 而不是 round：世界点落在哪一格由"格子的左边界"决定
// ------------------------------------------------------------------
bool OccupancyGrid::worldToGrid(double wx, double wy, int & gx, int & gy) const
{
  constexpr double kEps = 1e-9;

  gx = static_cast<int>(std::floor((wx - origin_x_) / resolution_ + kEps));
  gy = static_cast<int>(std::floor((wy - origin_y_) / resolution_ + kEps));
  return inBounds(gx, gy);
}

void OccupancyGrid::gridToWorld(int gx, int gy, double & wx, double & wy) const
{
  wx = origin_x_ + (gx + 0.5) * resolution_;
  wy = origin_y_ + (gy + 0.5) * resolution_;
}

bool OccupancyGrid::inBounds(int gx, int gy) const
{
  return gx >= 0 && gx < width_ && gy >= 0 && gy < height_;
}

// ------------------------------------------------------------------
// 更新：log-odds 加法
// ------------------------------------------------------------------
void OccupancyGrid::updateOccupied(int gx, int gy)
{
  if (!inBounds(gx, gy)) {return;}
  const size_t i = static_cast<size_t>(gy) * width_ + gx;
  log_odds_[i] = std::min(lo_max_, log_odds_[i] + lo_occ_);
  observed_[i] = 1;
}

void OccupancyGrid::updateFree(int gx, int gy)
{
  if (!inBounds(gx, gy)) {return;}
  const size_t i = static_cast<size_t>(gy) * width_ + gx;
  log_odds_[i] = std::max(lo_min_, log_odds_[i] + lo_free_);
  observed_[i] = 1;
}

// ------------------------------------------------------------------
// 光线投射：Bresenham 直线算法
//   沿途所有格子 = free，终点 = occupied
// ------------------------------------------------------------------
void OccupancyGrid::raycast(double x0, double y0, double x1, double y1)
{
  int gx0, gy0, gx1, gy1;
  if (!worldToGrid(x0, y0, gx0, gy0)) {return;}
  if (!worldToGrid(x1, y1, gx1, gy1)) {return;}

  int dx = std::abs(gx1 - gx0);
  int dy = -std::abs(gy1 - gy0);
  const int sx = (gx0 < gx1) ? 1 : -1;
  const int sy = (gy0 < gy1) ? 1 : -1;
  int err = dx + dy;
  int x = gx0;
  int y = gy0;

  while (true) {
    if (x == gx1 && y == gy1) {
      updateOccupied(x, y);   // 终点：激光打到的障碍
      break;
    }
    updateFree(x, y);         // 沿途：激光穿过的自由空间

    const int e2 = 2 * err;
    if (e2 >= dy) {err += dy; x += sx;}
    if (e2 <= dx) {err += dx; y += sy;}
  }
}


void OccupancyGrid::setOccupancyValue(int gx, int gy, int8_t value)
{
  if (!inBounds(gx, gy)) {return;}
  const size_t i = static_cast<size_t>(gy) * width_ + gx;
  if (value < 0) {
    log_odds_[i] = 0.0;
    observed_[i] = 0;          // 未知：既没观测过，也没有对数几率
  } else if (value >= 50) {
    log_odds_[i] = lo_max_;    // 占据：直接拉到上限
    observed_[i] = 1;
  } else {
    log_odds_[i] = lo_min_;    // 空闲：直接拉到下限
    observed_[i] = 1;
  }
}

// ------------------------------------------------------------------
// 读取
// ------------------------------------------------------------------
double OccupancyGrid::logOdds(int gx, int gy) const
{
  if (!inBounds(gx, gy)) {return 0.0;}
  return log_odds_[static_cast<size_t>(gy) * width_ + gx];
}

bool OccupancyGrid::observed(int gx, int gy) const
{
  if (!inBounds(gx, gy)) {return false;}
  return observed_[static_cast<size_t>(gy) * width_ + gx] != 0;
}

int8_t OccupancyGrid::probability(int gx, int gy) const
{
  if (!inBounds(gx, gy) || !observed(gx, gy)) {return -1;}
  const double l = logOdds(gx, gy);

  // log-odds → 概率：  p = 1 - 1/(1 + exp(l))  =  exp(l)/(1+exp(l))
  const double p = 1.0 - 1.0 / (1.0 + std::exp(l));
  return static_cast<int8_t>(std::lround(std::clamp(p, 0.0, 1.0) * 100.0));
}

std::vector<int8_t> OccupancyGrid::toOccupancyData() const
{
  std::vector<int8_t> out(static_cast<size_t>(width_) * height_, -1);
  for (int gy = 0; gy < height_; ++gy) {
    for (int gx = 0; gx < width_; ++gx) {
      out[static_cast<size_t>(gy) * width_ + gx] = probability(gx, gy);
    }
  }
  return out;
}

void OccupancyGrid::setLogOddsParams(double occ, double free, double min_lo, double max_lo)
{
  lo_occ_ = occ;
  lo_free_ = free;
  lo_min_ = min_lo;
  lo_max_ = max_lo;
}

}  // namespace mini_slam