#include "mini_slam/occupancy_grid.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <string>

using mini_slam::OccupancyGrid;

// ============================================================
// 测试 1：坐标转换（用你真实地图的参数）
//   my_map.yaml: resolution 0.05, origin [-2.75, -2.025], 110 x 81
// ============================================================
TEST(OccupancyGrid, CoordinateConversion)
{
  OccupancyGrid g(0.05, -2.75, -2.025, 110, 81);

  int gx = -1, gy = -1;

  // 世界原点 (0,0) → 像素 (55, 40)
  ASSERT_TRUE(g.worldToGrid(0.0, 0.0, gx, gy));
  EXPECT_EQ(gx, 55);
  EXPECT_EQ(gy, 40);

  // 栅格 (0,0) 的中心 = origin + 0.5*res
  double wx = 0.0, wy = 0.0;
  g.gridToWorld(0, 0, wx, wy);
  EXPECT_NEAR(wx, -2.75 + 0.025, 1e-9);
  EXPECT_NEAR(wy, -2.025 + 0.025, 1e-9);

  // 边界外
  EXPECT_FALSE(g.worldToGrid(-3.0, 0.0, gx, gy));
  EXPECT_FALSE(g.worldToGrid(0.0, -3.0, gx, gy));

  // 右上角边界内
  EXPECT_TRUE(g.worldToGrid(2.749, 2.024, gx, gy));
  EXPECT_EQ(gx, 109);
  EXPECT_EQ(gy, 80);
}

// ============================================================
// 测试 2：log-odds 累加
// ============================================================
TEST(OccupancyGrid, LogOddsAccumulation)
{
  OccupancyGrid g(0.05, 0.0, 0.0, 20, 20);

  EXPECT_FALSE(g.observed(5, 5));
  EXPECT_EQ(g.probability(5, 5), -1);      // 未观测

  g.updateOccupied(5, 5);
  const double l1 = g.logOdds(5, 5);
  EXPECT_NEAR(l1, 0.9, 1e-9);

  g.updateOccupied(5, 5);
  EXPECT_NEAR(g.logOdds(5, 5), 1.8, 1e-9);

  g.updateOccupied(5, 5);
  EXPECT_NEAR(g.logOdds(5, 5), 2.7, 1e-9);

  // p = 1 - 1/(1+e^2.7) ≈ 0.937  → 94
  EXPECT_EQ(g.probability(5, 5), 94);

  // clamp 上限 3.5
  for (int i = 0; i < 10; ++i) {g.updateOccupied(5, 5);}
  EXPECT_NEAR(g.logOdds(5, 5), 3.5, 1e-9);
}

// ============================================================
// 测试 3：光线投射
// ============================================================
TEST(OccupancyGrid, Raycast)
{
  OccupancyGrid g(0.05, 0.0, 0.0, 100, 100);

  // 从 (0.025, 0.025)（格 0,0 中心）打到 (0.525, 0.025)（格 10,0 中心）
  g.raycast(0.025, 0.025, 0.525, 0.025);

  // 起点应该是空闲（被标记 free 一次）
  EXPECT_TRUE(g.observed(0, 0));
  EXPECT_LT(g.logOdds(0, 0), 0.0);

  // 中间格为空闲
  EXPECT_LT(g.logOdds(5, 0), 0.0);

  // 终点为占据
  EXPECT_GT(g.logOdds(10, 0), 0.0);
  EXPECT_GT(g.probability(10, 0), 50);

  // 射线没经过的地方仍未观测
  EXPECT_FALSE(g.observed(5, 5));
}

// ============================================================
// 测试 4：toOccupancyData 的三值
// ============================================================
TEST(OccupancyGrid, OccupancyDataValues)
{
  OccupancyGrid g(0.05, 0.0, 0.0, 10, 10);
  g.updateOccupied(1, 1);
  g.updateFree(2, 2);

  const auto d = g.toOccupancyData();
  ASSERT_EQ(d.size(), 100u);

  EXPECT_GT(d[1 * 10 + 1], 50);     // 占据
  EXPECT_LT(d[2 * 10 + 2], 50);     // 空闲
  EXPECT_EQ(d[9 * 10 + 9], -1);     // 未知
}

// ============================================================
// 测试 5：画出来看看（用 ASCII 打印地图）
// ============================================================
TEST(OccupancyGrid, AsciiRender)
{
  OccupancyGrid g(0.1, 0.0, 0.0, 40, 20);

  // 画一堵竖墙 x = 2.0，y 从 0.5 到 1.5
  for (int i = 0; i <= 10; ++i) {
    const double y = 0.5 + i * 0.1;
    g.raycast(0.5, 1.0, 2.0, y);       // 从左边某点射向墙
  }

  std::printf("\n");
  for (int gy = g.height() - 1; gy >= 0; --gy) {
    std::string row;
    for (int gx = 0; gx < g.width(); ++gx) {
      if (!g.observed(gx, gy)) {
        row += ' ';
      } else if (g.logOdds(gx, gy) > 0.0) {
        row += '#';                      // 障碍
      } else {
        row += '.';                      // 自由
      }
    }
    std::printf("  |%s|\n", row.c_str());
  }
  std::printf("\n");
}


// ============================================================
// 回归 1：射线打到墙上，墙必须连续，不能有洞
//   曾因 0.7/0.1 = 6.99999999999999911 导致 floor 算错，出现缺口
// ============================================================
TEST(OccupancyGrid, RaycastWallIsContinuous)
{
  OccupancyGrid g(0.1, 0.0, 0.0, 40, 20);

  for (int i = 0; i <= 10; ++i) {
    const double y = 0.5 + i * 0.1;
    g.raycast(0.5, 1.0, 2.0, y);
  }

  const int wall_gx = 20;
  int missing = 0;
  for (int gy = 5; gy <= 15; ++gy) {
    if (g.logOdds(wall_gx, gy) <= 0.0) {
      ++missing;
      std::printf("  [MISSING] wall cell (%d, %d) logOdds=%.3f\n",
                  wall_gx, gy, g.logOdds(wall_gx, gy));
    }
  }
  EXPECT_EQ(missing, 0) << "wall has " << missing << " holes (float div error)";
}

// ============================================================
// 回归 2：落在格子边界上的世界点，必须归到【右边那一格】
//   0.7/0.1 = 6.99999999999999911 -> 不加 epsilon 会算成 6
// ============================================================
TEST(OccupancyGrid, BoundaryRoundingIsStable)
{
  OccupancyGrid g(0.1, 0.0, 0.0, 40, 20);

  struct Case { double wy; int expect_gy; };
  const Case cases[] = {
    {0.5, 5}, {0.6, 6}, {0.7, 7}, {0.8, 8},
    {1.2, 12}, {1.4, 14}, {1.9, 19},
  };
  for (const auto & c : cases) {
    int gx = -1, gy = -1;
    ASSERT_TRUE(g.worldToGrid(0.0, c.wy, gx, gy))
      << "world y=" << c.wy << " unexpectedly out of bounds";
    EXPECT_EQ(gy, c.expect_gy)
      << "world y=" << c.wy << " expect gy=" << c.expect_gy
      << " actual gy=" << gy << " (float error)";
  }

  // 上边界恰好在外（gy = 20 越界，height = 20）
  int gx = -1, gy = -1;
  EXPECT_FALSE(g.worldToGrid(0.0, 2.0, gx, gy))
    << "world y=2.0 应该越界";
  EXPECT_EQ(gy, 20);
}

// ============================================================
// 回归 3：边界点不能掉进前一个格子
// ============================================================
TEST(OccupancyGrid, BoundaryNotFallingToPreviousCell)
{
  OccupancyGrid g(0.1, 0.0, 0.0, 40, 20);

  const double tricky[] = {0.6, 0.7, 1.2, 1.4, 1.7, 1.9};
  for (double wy : tricky) {
    int gx = -1, gy = -1;
    ASSERT_TRUE(g.worldToGrid(0.0, wy, gx, gy)) << "y=" << wy;
    const double center_wy = gy * 0.1 + 0.05;
    EXPECT_GT(center_wy, wy - 0.1)
      << "y=" << wy << " 掉进了前一个格子";
  }
}