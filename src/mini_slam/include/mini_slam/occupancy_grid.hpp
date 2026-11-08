#ifndef MINI_SLAM__OCCUPANCY_GRID_HPP_
#define MINI_SLAM__OCCUPANCY_GRID_HPP_

#include <cstdint>
#include <vector>

namespace mini_slam
{

/// 二维占据栅格地图（对数几率 log-odds 表示）
///
/// 为什么用 log-odds 而不是概率：
///   概率做贝叶斯更新要连乘，多次观测后数值会下溢到 0；
///   取对数后变成"加法"，可以无限累加，再用上下限 clamp 就行。
class OccupancyGrid
{
public:
  OccupancyGrid(double resolution, double origin_x, double origin_y,
                 int width, int height);

  // ---------- 坐标转换 ----------
  bool worldToGrid(double wx, double wy, int & gx, int & gy) const;
  void gridToWorld(int gx, int gy, double & wx, double & wy) const;
  bool inBounds(int gx, int gy) const;

  // ---------- 更新 ----------
  void updateOccupied(int gx, int gy);
  void updateFree(int gx, int gy);

  /// 从 (x0,y0) 到 (x1,y1) 投射一条射线：
  /// 沿途格子标记为空闲，终点格子标记为占据
  void raycast(double x0, double y0, double x1, double y1);

  /// 直接设成某个 ROS 语义的值：100=占据，0=空闲，-1=未知
  ///   （map_io 读地图时用；raycast 是一次次累加，这里是直接赋值）
  void setOccupancyValue(int gx, int gy, int8_t value);

  

  // ---------- 读取 ----------
  double logOdds(int gx, int gy) const;
  bool observed(int gx, int gy) const;
  int8_t probability(int gx, int gy) const;  // 0~100，未观测返回 -1

  std::vector<int8_t> toOccupancyData() const;

  int width() const { return width_; }
  int height() const { return height_; }
  double resolution() const { return resolution_; }
  double originX() const { return origin_x_; }
  double originY() const { return origin_y_; }

  // ---------- 参数 ----------
  void setLogOddsParams(double occ, double free, double min_lo, double max_lo);

private:
  double resolution_;
  double origin_x_;
  double origin_y_;
  int width_;
  int height_;

  std::vector<double> log_odds_;   // width*height，初始 0（= p 0.5）
  std::vector<uint8_t> observed_;  // 是否被观测过

  double lo_occ_ = 0.9;
  double lo_free_ = -0.7;
  double lo_min_ = -2.0;
  double lo_max_ = 3.5;
};

}  // namespace mini_slam

#endif  // MINI_SLAM__OCCUPANCY_GRID_HPP_