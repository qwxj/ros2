#include "mini_slam/distance_field.hpp"

#include <algorithm>
#include <cmath>

namespace mini_slam
{

std::vector<float> buildDistanceField(const OccupancyGrid & map)
{
  const int w = map.width();
  const int h = map.height();
  if (w <= 0 || h <= 0) {return {};}

  const float kInf = 1e9f;
  const float kD1 = 1.0f;
  const float kD2 = 1.41421356f;

  std::vector<float> d(static_cast<size_t>(w) * h, kInf);

  // 障碍格距离 = 0
  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      if (map.probability(x, y) > 50) {
        d[static_cast<size_t>(y) * w + x] = 0.0f;
      }
    }
  }

  // 前向传播（左上 → 右下）
  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      const size_t i = static_cast<size_t>(y) * w + x;
      float v = d[i];
      if (x > 0) {v = std::min(v, d[i - 1] + kD1);}
      if (y > 0) {v = std::min(v, d[i - w] + kD1);}
      if (x > 0 && y > 0) {v = std::min(v, d[i - w - 1] + kD2);}
      if (x + 1 < w && y > 0) {v = std::min(v, d[i - w + 1] + kD2);}
      d[i] = v;
    }
  }

  // 后向传播（右下 → 左上）
  for (int y = h - 1; y >= 0; --y) {
    for (int x = w - 1; x >= 0; --x) {
      const size_t i = static_cast<size_t>(y) * w + x;
      float v = d[i];
      if (x + 1 < w) {v = std::min(v, d[i + 1] + kD1);}
      if (y + 1 < h) {v = std::min(v, d[i + w] + kD1);}
      if (x + 1 < w && y + 1 < h) {v = std::min(v, d[i + w + 1] + kD2);}
      if (x > 0 && y + 1 < h) {v = std::min(v, d[i + w - 1] + kD2);}
      d[i] = v;
    }
  }

  // 格子单位 → 米
  const float res = static_cast<float>(map.resolution());
  for (auto & v : d) {v *= res;}
  return d;
}

}  // namespace mini_slam
