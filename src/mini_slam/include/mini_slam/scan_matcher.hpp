#ifndef MINI_SLAM__SCAN_MATCHER_HPP_
#define MINI_SLAM__SCAN_MATCHER_HPP_

#include <vector>

#include "mini_slam/occupancy_grid.hpp"
#include "mini_slam/pose2d.hpp"

namespace mini_slam
{

struct ScanMatchResult
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
  double score{0.0};      // 0~1，1 = 完美贴合
  int candidates{0};      // 评估了多少个候选位姿
};

/// 相关性扫描匹配（Correlation Scan Matching 的最小实现）
///
/// 思路：
///   1. 预计算地图的【距离场】（每格到最近障碍的距离）
///   2. 在初始猜测附近枚举一堆候选位姿 (dx, dy, dtheta)
///   3. 对每个候选，把激光端点变换到世界系，查距离场算得分
///   4. 得分最高的就是匹配结果
///
/// 这就是 slam_toolbox 用的 CSM 的暴力版（它用多分辨率 + 分支定界加速）
class ScanMatcher
{
public:
  /// 设置要匹配的地图（会重建距离场）
  void setMap(const OccupancyGrid & map);

  /// 算某个位姿的得分（归一化到 0~1）
  double scoreAt(double x, double y, double yaw,
                 const std::vector<float> & ranges,
                 double angle_min, double angle_increment,
                 double range_max,
                 double sigma = 0.10) const;

  /// 在 initial guess 附近做由粗到细的搜索
  ScanMatchResult match(const std::vector<float> & ranges,
                        double angle_min, double angle_increment,
                        double range_max,
                        double guess_x, double guess_y, double guess_yaw,
                        double search_xy = 0.20,
                        double search_yaw = 0.20,
                        int steps = 7,
                        double sigma = 0.10) const;

  /// 设置激光安装外参（laser_link 相对 base_link）：
  ///   x/y 是激光在车体坐标系下的位置，yaw 是激光相对车头的旋转。
  ///   scoreAt / match 传进来的位姿是【base_link】的，内部会先变换到 laser_link
  ///   再算每一束的端点。不设置时按 0 处理（等于假设激光就在车体中心）。
  ///
  ///   为什么需要它：仿真里激光装在车头前方 0.10 m，若忽略这个偏移，
  ///   建图时整张图会随车头朝向被抹开约 0.1 m，转几次弯后墙就变成放射状条纹。
  void setLaserOffset(double x, double y, double yaw)
  {
    laser_x_ = x; laser_y_ = y; laser_yaw_ = yaw;
  }

  bool hasMap() const { return has_map_; }
  int mapWidth() const { return map_w_; }
  int mapHeight() const { return map_h_; }

private:
  double rawScore(double wx, double wy, double yaw,
                  const std::vector<float> & ranges,
                  double angle_min, double angle_increment,
                  double range_max,
                  double sigma) const;

  std::vector<float> dist_field_;
  int map_w_{0};
  int map_h_{0};
  double map_res_{0.05};
  double map_origin_x_{0.0};
  double map_origin_y_{0.0};
  bool has_map_{false};

  // 激光安装外参（laser_link 相对 base_link）
  double laser_x_{0.0};
  double laser_y_{0.0};
  double laser_yaw_{0.0};
};

}  // namespace mini_slam

#endif