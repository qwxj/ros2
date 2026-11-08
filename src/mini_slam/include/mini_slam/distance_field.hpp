#ifndef MINI_SLAM__DISTANCE_FIELD_HPP_
#define MINI_SLAM__DISTANCE_FIELD_HPP_

#include <vector>

#include "mini_slam/occupancy_grid.hpp"

namespace mini_slam
{

/// 两遍 chamfer 距离变换：返回每格到【最近占据格】的距离（米）
///
/// 复杂度 O(W*H)，而暴力版是 O(W*H*障碍数)。
/// 用于 AMCL 的似然场打分和扫描匹配的评分——两者每个控制周期都要用，
/// 暴力版在 400x400 的图上要几百毫秒，chamfer 只要零点几毫秒。
///
/// 权重 1 / sqrt(2) 是 chamfer 的标准近似，最大误差约 2%。
std::vector<float> buildDistanceField(const OccupancyGrid & map);

}  // namespace mini_slam

#endif
