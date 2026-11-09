#include "mini_slam/occupancy_grid.hpp"
#include "mini_slam/particle_filter.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

using mini_slam::OccupancyGrid;
using mini_slam::ParticleFilter;

using mini_slam::Pose2D;

static const double kAngleMinAdv = -M_PI;
static const double kAngleIncAdv = 2.0 * M_PI / 360.0;


// ---------- 工具：从 (x,y,angle) 射线，返回撞到占据格的距离 ----------
static double castRay(const OccupancyGrid & map, double x, double y, double angle,
                      double max_range, double step)
{
  for (double r = 0.0; r < max_range; r += step) {
    int gx = 0, gy = 0;
    if (!map.worldToGrid(x + r * std::cos(angle), y + r * std::sin(angle), gx, gy)) {
      return max_range;
    }
    if (map.probability(gx, gy) > 50) {return r;}
  }
  return max_range;
}

// ---------- 工具：在给定位姿下生成一帧合成激光 ----------
static std::vector<float> makeScan(const OccupancyGrid & map, double x, double y,
                                   double yaw, int beams, double angle_min,
                                   double angle_inc, double max_range)
{
  std::vector<float> out(static_cast<size_t>(beams));
  for (int i = 0; i < beams; ++i) {
    const double a = yaw + angle_min + i * angle_inc;
    out[i] = static_cast<float>(castRay(map, x, y, a, max_range, 0.02));
  }
  return out;
}

// ---------- 造一个 4m x 4m 四面有墙的房间 ----------
static OccupancyGrid makeRoom()
{
    OccupancyGrid m(0.1, 0.0, 0.0, 40, 40);
  for (int gx = 0; gx < 40; ++gx) {
    m.updateOccupied(gx, 0);
    m.updateOccupied(gx, 39);
  }
  for (int gy = 0; gy < 40; ++gy) {
    m.updateOccupied(0, gy);
    m.updateOccupied(39, gy);
  }
  return m;
}

TEST(ParticleFilter, InitializeScattersAroundPrior)
{
  ParticleFilter pf(2000, 42);
  EXPECT_FALSE(pf.initialized());

  pf.initialize(1.0, 2.0, 0.5, 0.3, 0.2);
  EXPECT_TRUE(pf.initialized());
  EXPECT_EQ(pf.numParticles(), 2000);

  const auto est = pf.estimatedPose();
  EXPECT_NEAR(est.x, 1.0, 0.05);
  EXPECT_NEAR(est.y, 2.0, 0.05);
  EXPECT_NEAR(est.yaw, 0.5, 0.05);
}

TEST(ParticleFilter, PredictMovesAllParticles)
{
  ParticleFilter pf(1000, 7);
  pf.initialize(0.0, 0.0, 0.0, 1e-6, 1e-6);   // 几乎不散开

  pf.predict(1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);   // 全零噪声 → 精确平移
  const auto est = pf.estimatedPose();
  EXPECT_NEAR(est.x, 1.0, 1e-3);
  EXPECT_NEAR(est.y, 0.0, 1e-3);
  EXPECT_NEAR(est.yaw, 0.0, 1e-3);
}

TEST(ParticleFilter, EffectiveSampleSizeDropsAfterWeighting)
{
  const OccupancyGrid map = makeRoom();
  const double angle_min = -M_PI;
  const double angle_inc = 2.0 * M_PI / 360.0;
  const double max_range = 4.0;

  ParticleFilter pf(2000, 3);
  pf.initialize(2.0, 2.0, 0.0, 0.5, 0.3);

  const double neff0 = pf.effectiveSampleSize();
  EXPECT_NEAR(neff0, 2000.0, 1.0) << "初始化后权重均匀，Neff 应该约等于粒子数";

  const auto scan = makeScan(map, 2.0, 2.0, 0.0, 360, angle_min, angle_inc, max_range);
  pf.setMap(map);
  pf.updateWeights(scan, angle_min, angle_inc, max_range, 0.15);

  const double neff1 = pf.effectiveSampleSize();
  EXPECT_LT(neff1, neff0 * 0.5) << "打完分后权重集中，Neff 应该明显下降";
}

TEST(ParticleFilter, ResampleMakesWeightsUniform)
{
  const OccupancyGrid map = makeRoom();
  const double angle_min = -M_PI;
  const double angle_inc = 2.0 * M_PI / 360.0;
  const double max_range = 4.0;

  ParticleFilter pf(1000, 11);
  pf.initialize(2.0, 2.0, 0.0, 0.5, 0.3);

  const auto scan = makeScan(map, 2.0, 2.0, 0.0, 360, angle_min, angle_inc, max_range);
  pf.setMap(map);
  pf.updateWeights(scan, angle_min, angle_inc, max_range, 0.15);
  pf.resample();

  for (const auto & p : pf.particles()) {
    EXPECT_NEAR(p.weight, 1.0 / 1000.0, 1e-12);
  }
  EXPECT_NEAR(pf.effectiveSampleSize(), 1000.0, 1.0);
}

// ============================================================
// 端到端：粒子云从散开收敛到真实位姿（这就是 AMCL 在干的事）
// ============================================================
TEST(ParticleFilter, ConvergesToTruePose)
{
  const OccupancyGrid map = makeRoom();
  const double angle_min = -M_PI;
  const double angle_inc = 2.0 * M_PI / 360.0;
  const double max_range = 4.0;

  const double true_x = 2.0, true_y = 2.0, true_yaw = 0.0;
  const auto scan = makeScan(map, true_x, true_y, true_yaw, 360,
                             angle_min, angle_inc, max_range);

  ParticleFilter pf(3000, 2024);
  pf.initialize(2.0, 2.0, 0.0, 1.0, 0.6);   // 初始很不确定
  pf.setMap(map);

  const auto before = pf.estimatedPose();
  std::printf("  before: (%.3f, %.3f, %.3f)\n", before.x, before.y, before.yaw);

  for (int it = 0; it < 15; ++it) {
    pf.updateWeights(scan, angle_min, angle_inc, max_range, 0.15);
    pf.resample();
  }

  const auto after = pf.estimatedPose();
  std::printf("  after : (%.3f, %.3f, %.3f)\n", after.x, after.y, after.yaw);

  int near = 0;
  for (const auto & p : pf.particles()) {
    if (std::hypot(p.pose.x - true_x, p.pose.y - true_y) < 0.15) {++near;}
  }
  std::printf("  count of particles within 0.15m: %d / %d\n", near, pf.numParticles());

  EXPECT_NEAR(after.x, true_x, 0.25);
  EXPECT_NEAR(after.y, true_y, 0.25);
  EXPECT_GT(near, pf.numParticles() / 2);
  EXPECT_TRUE(pf.converged(0.4, 0.3));
}

// ============================================================
// C-1：KLD 粒子数随"占据格子数"增长
// ============================================================
TEST(ParticleFilterAdvanced, KldSampleSizeGrowsWithBins)
{
  ParticleFilter pf(5000, 1);
  pf.setKldParams(0.05, 0.99, 50, 0.1, 0.1);

  const double n2  = pf.kldSampleSize(2);
  const double n10 = pf.kldSampleSize(10);
  const double n50 = pf.kldSampleSize(50);

  std::printf("  k=2 -> n=%.1f   k=10 -> n=%.1f   k=50 -> n=%.1f\n", n2, n10, n50);

  EXPECT_LT(n2, n10);
  EXPECT_LT(n10, n50);
  EXPECT_GT(n2, 0.0);
}

// ============================================================
// C-2：收敛后 KLD 会自动减少粒子数（省算力）
// ============================================================
TEST(ParticleFilterAdvanced, KldReducesParticlesAfterConvergence)
{
  const OccupancyGrid map = makeRoom();
  ParticleFilter pf(3000, 77);
  pf.setKldParams(0.05, 0.99, 100, 0.10, 0.10);
  pf.initialize(2.0, 2.0, 0.0, 1.0, 0.6);
  pf.setMap(map);

  // 关键：在【还没打过分】时测量，此时云最散
  const int k_spread = pf.countOccupiedBins(0.10, 0.10);
  const double n_spread = pf.kldSampleSize(k_spread);

  const auto scan = makeScan(map, 2.0, 2.0, 0.0, 360, kAngleMinAdv, kAngleIncAdv, 4.0);
  for (int i = 0; i < 15; ++i) {
    pf.updateWeights(scan, kAngleMinAdv, kAngleIncAdv, 4.0);
    pf.resample();
  }

  const int k_conv = pf.countOccupiedBins(0.10, 0.10);
  const double n_conv = pf.kldSampleSize(k_conv);
  const int n_actual = pf.lastResampleCount();

  std::printf("  spread   : bins=%4d  kld_n=%.0f\n", k_spread, n_spread);
  std::printf("  converged: bins=%4d  kld_n=%.0f  (clamped to %d)\n",
              k_conv, n_conv, n_actual);

  EXPECT_GT(k_spread, k_conv);
  EXPECT_GT(n_spread, n_conv);
  EXPECT_GE(n_actual, 100);
  EXPECT_LE(n_actual, 3000);
}

// ============================================================
// C-3：注入默认关闭
// ============================================================
TEST(ParticleFilterAdvanced, InjectionDisabledByDefault)
{
  const OccupancyGrid map = makeRoom();
  ParticleFilter pf(1000, 5);
  pf.initialize(2.0, 2.0, 0.0, 0.3, 0.3);
  pf.setMap(map);

  const auto scan = makeScan(map, 2.0, 2.0, 0.0, 360, kAngleMinAdv, kAngleIncAdv, 4.0);
  pf.updateWeights(scan, kAngleMinAdv, kAngleIncAdv, 4.0);
  pf.resample();

  EXPECT_NEAR(pf.injectionProbability(), 0.0, 1e-12);
}

// ============================================================
// C-4：绑架场景 —— 观测突然不匹配时，注入概率飙升
// ============================================================
TEST(ParticleFilterAdvanced, InjectionRisesWhenKidnapped)
{
  const OccupancyGrid map = makeRoom();
  ParticleFilter pf(1000, 99);
  pf.setRecoveryParams(0.001, 0.1);
  pf.setUniformPoseFn([]() {
    std::uniform_real_distribution<double> ux(0.2, 3.8);
    std::uniform_real_distribution<double> uy(0.2, 3.8);
    std::uniform_real_distribution<double> ua(-M_PI, M_PI);
    static std::mt19937 rng(1234);
    Pose2D p;
    p.x = ux(rng); p.y = uy(rng); p.yaw = ua(rng);
    return p;
  });
  pf.initialize(2.0, 2.0, 0.0, 0.3, 0.3);
  pf.setMap(map);

  const auto good = makeScan(map, 2.0, 2.0, 0.0, 360, kAngleMinAdv, kAngleIncAdv, 4.0);

  for (int i = 0; i < 5; ++i) {
    pf.updateWeights(good, kAngleMinAdv, kAngleIncAdv, 4.0);
    pf.resample();
  }
  const double p_good = pf.injectionProbability();

  std::vector<float> bad(360, 1.0f);      // 全都报 1.0 m，和地图完全不符
  for (int i = 0; i < 3; ++i) {
    pf.updateWeights(bad, kAngleMinAdv, kAngleIncAdv, 4.0);
    pf.resample();
  }
  const double p_bad = pf.injectionProbability();

  std::printf("  injection after good scans: %.6f\n", p_good);
  std::printf("  injection after bad  scans: %.6f\n", p_bad);

  EXPECT_LT(p_good, 0.3);
  EXPECT_GT(p_bad, 0.5);
}



// ============================================================
// C-5：poseCovariance() —— 收敛后方差应该显著变小
// ============================================================
TEST(ParticleFilterAdvanced, PoseCovarianceShrinksAfterConvergence)
{
  const OccupancyGrid map = makeRoom();
  ParticleFilter pf(2000, 424242);
  pf.initialize(2.0, 2.0, 0.0, 1.0, 0.6);      // 初始很散
  pf.setMap(map);

  double vx0 = 0.0, vy0 = 0.0, vw0 = 0.0;
  pf.poseCovariance(vx0, vy0, vw0);
  std::printf("  初始: var=(%.4f, %.4f, %.4f)\n", vx0, vy0, vw0);

  const auto scan = makeScan(map, 2.0, 2.0, 0.0, 360, kAngleMinAdv, kAngleIncAdv, 4.0);
  for (int i = 0; i < 15; ++i) {
    pf.updateWeights(scan, kAngleMinAdv, kAngleIncAdv, 4.0);
    pf.resample();
  }

  double vx1 = 0.0, vy1 = 0.0, vw1 = 0.0;
  pf.poseCovariance(vx1, vy1, vw1);
  std::printf("  收敛: var=(%.6f, %.6f, %.6f)\n", vx1, vy1, vw1);

  EXPECT_GT(vx0, vx1) << "收敛后 x 方差应该变小";
  EXPECT_GT(vy0, vy1) << "收敛后 y 方差应该变小";
  EXPECT_GT(vw0, vw1) << "收敛后 yaw 方差应该变小";
  EXPECT_GE(vx1, 0.0);
  EXPECT_GE(vy1, 0.0);
  EXPECT_GE(vw1, 0.0);
}