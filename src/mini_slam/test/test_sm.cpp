#include "mini_slam/occupancy_grid.hpp"
#include "mini_slam/scan_matcher.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

using mini_slam::OccupancyGrid;
using mini_slam::ScanMatcher;

static const double kAngleMin = -M_PI;
static const double kAngleInc = 2.0 * M_PI / 360.0;
static const double kMaxRange = 5.0;

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

static std::vector<float> makeScan(const OccupancyGrid & map, double x, double y,
                                   double yaw, int beams = 360)
{
  std::vector<float> out(static_cast<size_t>(beams));
  for (int i = 0; i < beams; ++i) {
    out[i] = static_cast<float>(
      castRay(map, x, y, yaw + kAngleMin + i * kAngleInc, kMaxRange, 0.02));
  }
  return out;
}

// 4m x 4m 房间，墙在 0.1 和 3.9
static OccupancyGrid makeRoom(double res = 0.05)
{
  const int n = static_cast<int>(std::lround(4.0 / res));
  OccupancyGrid m(res, 0.0, 0.0, n, n);
  for (int gx = 0; gx < n; ++gx) {
    m.updateOccupied(gx, 0);
    m.updateOccupied(gx, n - 1);
  }
  for (int gy = 0; gy < n; ++gy) {
    m.updateOccupied(0, gy);
    m.updateOccupied(n - 1, gy);
  }
  return m;
}

TEST(ScanMatcher, ScoreIsMaxAtTruePose)
{
  const OccupancyGrid room = makeRoom();
  const auto scan = makeScan(room, 2.0, 2.0, 0.3);

  ScanMatcher sm;
  sm.setMap(room);

  const double s_true = sm.scoreAt(2.0, 2.0, 0.3, scan, kAngleMin, kAngleInc, kMaxRange);
  const double s_off  = sm.scoreAt(2.3, 2.0, 0.3, scan, kAngleMin, kAngleInc, kMaxRange);
  const double s_off2 = sm.scoreAt(2.0, 2.0, 0.6, scan, kAngleMin, kAngleInc, kMaxRange);

  std::printf("  score(true)=%.4f  score(dx=0.3)=%.4f  score(dyaw=0.3)=%.4f\n",
              s_true, s_off, s_off2);

  EXPECT_GT(s_true, 0.9) << "真位姿下应该非常贴合";
  EXPECT_LT(s_off, s_true * 0.7) << "平移 0.3m 应该明显掉分";
  EXPECT_LT(s_off2, s_true * 0.9) << "偏转 0.3rad 应该掉分";
}

TEST(ScanMatcher, RecoversKnownOffset)
{
  const OccupancyGrid room = makeRoom();
  const double tx = 2.0, ty = 2.0, tyaw = 0.3;
  const auto scan = makeScan(room, tx, ty, tyaw);

  ScanMatcher sm;
  sm.setMap(room);

  const auto r = sm.match(scan, kAngleMin, kAngleInc, kMaxRange,
                          tx + 0.12, ty - 0.08, tyaw + 0.06,
                          0.20, 0.20, 7);

  std::printf("  matched: (%.4f, %.4f, %.4f) score=%.4f candidates=%d\n",
              r.x, r.y, r.yaw, r.score, r.candidates);

  EXPECT_NEAR(r.x, tx, 0.05);
  EXPECT_NEAR(r.y, ty, 0.05);
  EXPECT_NEAR(r.yaw, tyaw, 0.05);
  EXPECT_GT(r.score, 0.9);
}

// ============================================================
// 端到端：从零建图（SLAM 的核心闭环）
// ============================================================
TEST(ScanMatcher, BuildMapFromScratch)
{
  const OccupancyGrid truth_room = makeRoom(0.05);

  OccupancyGrid slam_map(0.05, -1.0, -1.0, 120, 120);

  ScanMatcher sm;

  double px = 1.0, py = 2.0, pyaw = 0.0;
  double pose_err_max = 0.0;
  int matched_ok = 0;

  const int kSteps = 9;
  for (int k = 0; k < kSteps; ++k) {
    const double tx = 1.0 + 0.2 * k;
    const double ty = 2.0;
    const double tyaw = 0.0;

    const auto scan = makeScan(truth_room, tx, ty, tyaw);

    double nx, ny, nyaw;
    if (k == 0) {
      nx = tx; ny = ty; nyaw = tyaw;      // 第一帧：地图空，直接放
    } else {
      const double guess_x = px + 0.21;    // odom 5% 尺度误差
      sm.setMap(slam_map);
      const auto r = sm.match(scan, kAngleMin, kAngleInc, kMaxRange,
                              guess_x, py, pyaw, 0.15, 0.10, 7);
      nx = r.x; ny = r.y; nyaw = r.yaw;
      if (std::hypot(nx - tx, ny - ty) < 0.10) {++matched_ok;}
    }

    for (size_t i = 0; i < scan.size(); ++i) {
      const double r = scan[i];
      if (!std::isfinite(r) || r >= kMaxRange) {continue;}
      const double a = nyaw + kAngleMin + static_cast<double>(i) * kAngleInc;
      slam_map.raycast(nx, ny, nx + r * std::cos(a), ny + r * std::sin(a));
    }

    px = nx; py = ny; pyaw = nyaw;
    const double err = std::hypot(px - tx, py - ty);
    pose_err_max = std::max(pose_err_max, err);
  }

  int occupied = 0;
  for (int gy = 0; gy < slam_map.height(); ++gy) {
    for (int gx = 0; gx < slam_map.width(); ++gx) {
      if (slam_map.probability(gx, gy) > 50) {++occupied;}
    }
  }

  std::printf("  final pose: (%.3f, %.3f, %.3f), truth: (%.3f, %.3f, 0.000)\n",
              px, py, pyaw, 1.0 + 0.2 * (kSteps - 1), 2.0);
  std::printf("  max pose error over run: %.4f m\n", pose_err_max);
  std::printf("  matched frames: %d / %d\n", matched_ok, kSteps - 1);
  std::printf("  occupied cells built: %d\n", occupied);

  EXPECT_LT(pose_err_max, 0.10) << "扫描匹配应该把 odom 漂移纠回来";
  EXPECT_GE(matched_ok, kSteps - 2) << "绝大多数帧都应该匹配成功";
  EXPECT_GT(occupied, 200) << "地图上应该建出墙来";

  std::printf("\n");
  for (int gy = slam_map.height() - 1; gy >= 0; gy -= 2) {
    std::string row;
    for (int gx = 0; gx < slam_map.width(); gx += 1) {
      if (!slam_map.observed(gx, gy)) {row += ' ';}
      else if (slam_map.logOdds(gx, gy) > 0.0) {row += '#';}
      else {row += '.';}
    }
    std::printf("  |%s|\n", row.c_str());
  }
  std::printf("\n");
}