#include "mini_slam/particle_filter.hpp"

#include "mini_slam/distance_field.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <set>
#include <tuple>

namespace mini_slam
{

double ParticleFilter::wrapToPi(double a)
{
  return mini_slam::wrapToPi(a);
}

ParticleFilter::ParticleFilter(int max_particles, unsigned int seed)
: max_particles_(max_particles), rng_(seed)
{
}

void ParticleFilter::initialize(double x, double y, double yaw,
                                double sigma_xy, double sigma_yaw)
{
  std::normal_distribution<double> nx(0.0, sigma_xy);
  std::normal_distribution<double> ny(0.0, sigma_xy);
  std::normal_distribution<double> nyaw(0.0, sigma_yaw);

  particles_.assign(static_cast<size_t>(max_particles_), Particle{});
  for (auto & p : particles_) {
    p.pose.x = x + nx(rng_);
    p.pose.y = y + ny(rng_);
    p.pose.yaw = wrapToPi(yaw + nyaw(rng_));
    p.weight = 1.0 / max_particles_;
  }
  initialized_ = true;
  recovery_initialized_ = false;
  log_w_slow_ = 0.0;
  log_w_fast_ = 0.0;
}

void ParticleFilter::predict(double dx, double dy, double dyaw,
                             double alpha1, double alpha2,
                             double alpha3, double alpha4)
{
  if (!initialized_) {return;}

  const double delta_trans = std::hypot(dx, dy);
  double delta_rot1 = 0.0;
  if (delta_trans > 1e-6) {delta_rot1 = wrapToPi(std::atan2(dy, dx));}
  const double delta_rot2 = wrapToPi(dyaw - delta_rot1);

  for (auto & p : particles_) {
    const double var_rot1 = alpha1 * delta_rot1 * delta_rot1 +
      alpha2 * delta_trans * delta_trans;
    const double var_trans = alpha3 * delta_trans * delta_trans +
      alpha4 * delta_rot1 * delta_rot1 + alpha4 * delta_rot2 * delta_rot2;
    const double var_rot2 = alpha1 * delta_rot2 * delta_rot2 +
      alpha2 * delta_trans * delta_trans;

    std::normal_distribution<double> n1(0.0, std::max(std::sqrt(var_rot1), 1e-4));
    std::normal_distribution<double> n2(0.0, std::max(std::sqrt(var_trans), 1e-4));
    std::normal_distribution<double> n3(0.0, std::max(std::sqrt(var_rot2), 1e-4));

    const double rot1_hat = delta_rot1 + n1(rng_);
    const double trans_hat = delta_trans + n2(rng_);
    const double rot2_hat = delta_rot2 + n3(rng_);

    p.pose.x += trans_hat * std::cos(p.pose.yaw + rot1_hat);
    p.pose.y += trans_hat * std::sin(p.pose.yaw + rot1_hat);
    p.pose.yaw = wrapToPi(p.pose.yaw + rot1_hat + rot2_hat);
  }
}

void ParticleFilter::setMap(const OccupancyGrid & map)
{
  map_w_ = map.width();
  map_h_ = map.height();
  map_res_ = map.resolution();
  map_origin_x_ = map.originX();
  map_origin_y_ = map.originY();

  dist_field_ = buildDistanceField(map);
  has_map_ = true;
}

double ParticleFilter::logLikelihood(const Particle & p,
                                     const std::vector<float> & ranges,
                                     double angle_min, double angle_increment,
                                     double range_max,
                                     double sigma_hit,
                                     double max_dist) const
{
  // ★ AMCL 的 beam model（简化版）：
  //     p = z_hit·exp(-d²/2σ²) + z_rand·(1/range_max)
  //
  //   为什么必须有 z_rand 这一项：
  //     纯高斯下，一条"完全打偏"的光线贡献 log(exp(-22.2)) = -22.2，
  //     128 条光线就是 -2842，而完全正确只有 0。
  //     这个尖峰让权重瞬间被单个粒子吃掉 → 粒子瞬灭 → 一旦丢失就再也回不来。
  //     加上均匀项（对应传感器噪声/动态障碍），最差光线只值 -5.07，
  //     粒子云能保持多样性，过滤器才有恢复能力。
  const double inv_2sigma2 = 1.0 / (2.0 * sigma_hit * sigma_hit);
  const double z_hit = 0.95;
  const double z_rand = z_rand_;
  const double rand_term = z_rand / std::max(range_max, 1e-3);

  double log_l = 0.0;
  int used = 0;

  for (size_t i = 0; i < ranges.size(); ++i) {
    const double r = ranges[i];
    if (!std::isfinite(r) || r < 0.05 || r >= range_max) {continue;}

    const double a = p.pose.yaw + angle_min + static_cast<double>(i) * angle_increment;
    const double ex = p.pose.x + r * std::cos(a);
    const double ey = p.pose.y + r * std::sin(a);

    const int gx = static_cast<int>(std::floor((ex - map_origin_x_) / map_res_));
    const int gy = static_cast<int>(std::floor((ey - map_origin_y_) / map_res_));
    if (gx < 0 || gx >= map_w_ || gy < 0 || gy >= map_h_) {
      // 打到图外：按"最差"处理，但不能直接丢掉（丢掉会失去约束）
      log_l += std::log(rand_term);
      ++used;
      continue;
    }

    double d = dist_field_[static_cast<size_t>(gy) * map_w_ + gx];
    if (d > max_dist) {d = max_dist;}

    const double p_hit = std::exp(-d * d * inv_2sigma2);
    log_l += std::log(z_hit * p_hit + rand_term);
    ++used;
  }
  if (used == 0) {return 0.0;}
  return log_l;
}

void ParticleFilter::updateWeights(const std::vector<float> & ranges,
                                   double angle_min, double angle_increment,
                                   double range_max,
                                   double sigma_hit,
                                   double max_dist)
{
  if (!initialized_ || particles_.empty() || !has_map_) {return;}

  const size_t n = particles_.size();
  std::vector<double> log_w(n, 0.0);
  double max_log = -std::numeric_limits<double>::infinity();

  for (size_t i = 0; i < n; ++i) {
    log_w[i] = logLikelihood(particles_[i], ranges, angle_min, angle_increment,
                             range_max, sigma_hit, max_dist);
    max_log = std::max(max_log, log_w[i]);
  }
  if (!std::isfinite(max_log)) {return;}

  // ---------- 归一化（乘进旧权重）----------
  double sum = 0.0;
  for (size_t i = 0; i < n; ++i) {
    const double old_w = std::max(particles_[i].weight, 1e-300);
    const double combined = std::log(old_w) + (log_w[i] - max_log);
    particles_[i].weight = combined;
    sum += std::exp(combined);
  }
  if (sum <= 0.0) {return;}

  // ---------- recovery：未归一化的平均似然，在对数域上做慢/快平均 ----------
  // 对应 AMCL 的 pf_update_sensor：w_avg += sample->weight（在除以 total 之前累加）
  // 关键：w_fast / w_slow 是【比值】，所以绝对尺度会抵消，
  //       真正被检测到的是"似然相对长期均值掉了多少"。
  double sum_exp = 0.0;
  for (size_t i = 0; i < n; ++i) {sum_exp += std::exp(log_w[i] - max_log);}
  const double log_w_avg = std::log(sum_exp / static_cast<double>(n)) + max_log;

  if (!recovery_initialized_) {
    log_w_slow_ = log_w_avg;
    log_w_fast_ = log_w_avg;
    recovery_initialized_ = true;
  } else {
    log_w_slow_ += alpha_slow_ * (log_w_avg - log_w_slow_);
    log_w_fast_ += alpha_fast_ * (log_w_avg - log_w_fast_);
  }

  const double log_sum = std::log(sum);
  for (auto & p : particles_) {p.weight = std::exp(p.weight - log_sum);}
}

// ------------------------------------------------------------------
// KLD 自适应粒子数
// ------------------------------------------------------------------
void ParticleFilter::setKldParams(double kld_err, double kld_z, int min_particles,
                                  double bin_xy, double bin_yaw)
{
  kld_err_ = kld_err;
  kld_z_ = kld_z;
  min_particles_ = std::clamp(min_particles, 1, max_particles_);
  bin_xy_ = bin_xy;
  bin_yaw_ = bin_yaw;
  kld_enabled_ = true;
}

void ParticleFilter::disableKld() {kld_enabled_ = false;}

int ParticleFilter::countOccupiedBins(double bin_xy, double bin_yaw) const
{
  std::set<std::tuple<int, int, int>> bins;
  for (const auto & p : particles_) {
    if (p.weight <= 1e-15) {continue;}
    bins.insert(std::make_tuple(
      static_cast<int>(std::floor(p.pose.x / bin_xy)),
      static_cast<int>(std::floor(p.pose.y / bin_xy)),
      static_cast<int>(std::floor(wrapToPi(p.pose.yaw) / bin_yaw))));
  }
  return static_cast<int>(bins.size());
}

double ParticleFilter::kldSampleSize(int k) const
{
  // Fox 2003 的 KLD 采样公式：
  //   n = (k-1)/(2*eps) * (1 - 2/(9(k-1)) + sqrt(2/(9(k-1))) * z)^3
  if (k <= 1) {return static_cast<double>(std::max(min_particles_, 1));}
  const double a = static_cast<double>(k - 1);
  const double b = 2.0 / (9.0 * a);
  const double c = 1.0 - b + std::sqrt(b) * kld_z_;
  return a / (2.0 * kld_err_) * c * c * c;
}

// ------------------------------------------------------------------
// 随机注入
// ------------------------------------------------------------------
void ParticleFilter::setRecoveryParams(double alpha_slow, double alpha_fast)
{
  alpha_slow_ = alpha_slow;
  alpha_fast_ = alpha_fast;
}

void ParticleFilter::setUniformPoseFn(std::function<Pose2D()> fn)
{
  uniform_pose_fn_ = std::move(fn);
}

double ParticleFilter::injectionProbability() const
{
  if (!recovery_initialized_) {return 0.0;}
  const double ratio = std::exp(log_w_fast_ - log_w_slow_);   // w_fast / w_slow
  return std::clamp(1.0 - ratio, 0.0, 1.0);
}

void ParticleFilter::resample()
{
  if (!initialized_ || particles_.empty()) {return;}

  // ---------- 1. 决定新粒子数 ----------
  int n_new = max_particles_;
  if (kld_enabled_) {
    const int k = countOccupiedBins(bin_xy_, bin_yaw_);
    n_new = static_cast<int>(std::ceil(kldSampleSize(k)));
    n_new = std::clamp(n_new, min_particles_, max_particles_);
  }
  last_resample_count_ = n_new;

  // ---------- 2. 归一化防御 ----------
  double wsum = 0.0;
  for (const auto & p : particles_) {wsum += p.weight;}
  if (wsum <= 0.0) {return;}

  // ---------- 3. 注入概率 ----------
  const double w_diff = uniform_pose_fn_ ? injectionProbability() : 0.0;

  // ---------- 4. 系统重采样 + 注入 ----------
  const double step = 1.0 / static_cast<double>(n_new);
  std::uniform_real_distribution<double> uni_r(0.0, step);
  std::uniform_real_distribution<double> uni01(0.0, 1.0);
  const double r0 = uni_r(rng_);

  std::vector<Particle> out(static_cast<size_t>(n_new));
  double cum = particles_[0].weight / wsum;
  size_t idx = 0;

  for (int k = 0; k < n_new; ++k) {
    if (w_diff > 0.0 && uni01(rng_) < w_diff) {
      out[static_cast<size_t>(k)].pose = uniform_pose_fn_();
    } else {
      const double u = r0 + static_cast<double>(k) * step;
      while (u > cum && idx + 1 < particles_.size()) {
        ++idx;
        cum += particles_[idx].weight / wsum;
      }
      out[static_cast<size_t>(k)] = particles_[idx];
    }
    out[static_cast<size_t>(k)].weight = step;
  }

  particles_.swap(out);
}

double ParticleFilter::effectiveSampleSize() const
{
  double s = 0.0;
  for (const auto & p : particles_) {s += p.weight * p.weight;}
  return (s > 0.0) ? 1.0 / s : 0.0;
}



void ParticleFilter::poseCovariance(double & var_x, double & var_y, double & var_yaw) const
{
  var_x = var_y = var_yaw = 0.0;
  if (particles_.empty()) {return;}

  const Pose2D mean = estimatedPose();
  for (const auto & p : particles_) {
    const double dx = p.pose.x - mean.x;
    const double dy = p.pose.y - mean.y;
    // 朝向差要归一化到 [-pi, pi]，否则跨 ±pi 的粒子会被算成巨大方差
    const double dyaw = wrapToPi(p.pose.yaw - mean.yaw);

    var_x += p.weight * dx * dx;
    var_y += p.weight * dy * dy;
    var_yaw += p.weight * dyaw * dyaw;
  }
}


Pose2D ParticleFilter::estimatedPose() const
{
  Pose2D out;
  if (particles_.empty()) {return out;}

  double sx = 0.0, sy = 0.0, sw = 0.0, sc = 0.0, ss = 0.0;
  for (const auto & p : particles_) {
    const double w = p.weight;
    sx += w * p.pose.x;
    sy += w * p.pose.y;
    sc += w * std::cos(p.pose.yaw);
    ss += w * std::sin(p.pose.yaw);
    sw += w;
  }
  if (sw <= 0.0) {return out;}
  out.x = sx / sw;
  out.y = sy / sw;
  out.yaw = wrapToPi(std::atan2(ss, sc));
  return out;
}

bool ParticleFilter::converged(double dist_thresh, double yaw_thresh) const
{
  if (!initialized_ || particles_.empty()) {return false;}
  const Pose2D mean = estimatedPose();

  double var_xy = 0.0, sc = 0.0, ss = 0.0;
  for (const auto & p : particles_) {
    const double dx = p.pose.x - mean.x;
    const double dy = p.pose.y - mean.y;
    var_xy = std::max(var_xy, dx * dx + dy * dy);
    sc += std::cos(p.pose.yaw);
    ss += std::sin(p.pose.yaw);
  }
  const double yaw_mean = std::atan2(ss, sc);
  for (const auto & p : particles_) {
    if (std::abs(wrapToPi(p.pose.yaw - yaw_mean)) > yaw_thresh) {return false;}
  }
  return std::sqrt(var_xy) <= dist_thresh;
}

}  // namespace mini_slam