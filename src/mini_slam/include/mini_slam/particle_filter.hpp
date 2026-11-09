#ifndef MINI_SLAM__PARTICLE_FILTER_HPP_
#define MINI_SLAM__PARTICLE_FILTER_HPP_

#include <functional>
#include <random>
#include <vector>

#include "mini_slam/occupancy_grid.hpp"
#include "mini_slam/pose2d.hpp"

namespace mini_slam
{


struct Particle
{
  Pose2D pose;
  double weight{1.0};
};

/// 极简粒子滤波（AMCL 的核心算法）
///
/// 一次完整的 pf 循环：
///   predict()       用 odom 增量 + alpha 噪声移动所有粒子
///   updateWeights() 用激光 + 地图给每个粒子打分
///   resample()      按权重重新抽样
///
/// 进阶能力（对应 AMCL 的参数）：
///   setKldParams()       KLD 自适应粒子数（kld_err / kld_z）
///   setRecoveryParams()  随机注入（recovery_alpha_slow / fast）
class ParticleFilter
{
public:
  explicit ParticleFilter(int max_particles = 500, unsigned int seed = 42);

  void initialize(double x, double y, double yaw,
                  double sigma_xy, double sigma_yaw);

  void predict(double dx, double dy, double dyaw,
               double alpha1 = 0.05, double alpha2 = 0.01,
               double alpha3 = 0.05, double alpha4 = 0.01);

  void setMap(const OccupancyGrid & map);

  void updateWeights(const std::vector<float> & ranges,
                     double angle_min, double angle_increment,
                     double range_max,
                     double sigma_hit = 0.15,
                     double max_dist = 1.0);

  void resample();

  double effectiveSampleSize() const;

  /// 粒子云的加权协方差（只取对角项，单位是「方差」不是「标准差」）
  ///
  /// 用途：填 /amcl_pose 的 covariance 字段。
  ///   两级 EKF 用 (H·P·Hᵀ + R) 决定卡尔曼增益，R 就是观测协方差 ——
  ///   R 小 = 相信这次观测，R 大 = 更相信预测。
  ///   如果协方差写死，EKF 就永远用同一个权重，退化成固定比例的加权平均。
  ///
  /// 注意：调用方要自己加下限（如 1e-4）。粒子全部重合时方差为 0，
  ///       会让 EKF 认为"这次观测绝对准确"，恰恰是最危险的情况。
  void poseCovariance(double & var_x, double & var_y, double & var_yaw) const;
  
  Pose2D estimatedPose() const;
  bool converged(double dist_thresh, double yaw_thresh) const;

  // ---------------- KLD 自适应粒子数 ----------------
  void setKldParams(double kld_err, double kld_z, int min_particles,
                    double bin_xy = 0.10, double bin_yaw = 0.10);
  void disableKld();

  int countOccupiedBins(double bin_xy, double bin_yaw) const;
  double kldSampleSize(int k) const;
  int lastResampleCount() const { return last_resample_count_; }

  // ---------------- 随机注入（recovery）----------------
  void setRecoveryParams(double alpha_slow, double alpha_fast);

  /// beam model 里的均匀噪声权重（AMCL 的 laser_z_rand，默认 0.05）
  ///   调大 → 似然更平坦 → 粒子更多样、更不容易瞬灭，但定位精度略降
  void setBeamModelRand(double z_rand) {z_rand_ = z_rand;}
  void setUniformPoseFn(std::function<Pose2D()> fn);
  double injectionProbability() const;

  /// 当前慢/快平均似然（调试用）
  double wSlow() const {return log_w_slow_;}
  double wFast() const {return log_w_fast_;}

  const std::vector<Particle> & particles() const { return particles_; }
  int numParticles() const { return static_cast<int>(particles_.size()); }
  int maxParticles() const { return max_particles_; }
  bool initialized() const { return initialized_; }

  static double wrapToPi(double a);

private:
  double logLikelihood(const Particle & p,
                       const std::vector<float> & ranges,
                       double angle_min, double angle_increment,
                       double range_max,
                       double sigma_hit,
                       double max_dist) const;

  std::vector<Particle> particles_;
  int max_particles_;

  std::vector<float> dist_field_;
  int map_w_{0};
  int map_h_{0};
  double map_res_{0.05};
  double map_origin_x_{0.0};
  double map_origin_y_{0.0};
  bool has_map_{false};

  // KLD
  bool kld_enabled_{false};
  double kld_err_{0.05};
  double kld_z_{0.99};
  int min_particles_{50};
  double bin_xy_{0.10};
  double bin_yaw_{0.10};
  int last_resample_count_{0};

  // recovery：对数域上的慢/快平均似然（和 AMCL 的 pf_update_sensor 等价）
  double log_w_slow_{0.0};
  double log_w_fast_{0.0};
  double alpha_slow_{0.001};
  double alpha_fast_{0.1};
  bool recovery_initialized_{false};
  std::function<Pose2D()> uniform_pose_fn_;

  double z_rand_{0.05};

  std::mt19937 rng_;
  bool initialized_{false};
};

}  // namespace mini_slam

#endif