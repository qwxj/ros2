#include "mini_slam/pose_graph.hpp"

#include <gtest/gtest.h>

#include <cmath>

using mini_slam::PoseGraph;

TEST(PoseGraph, AddNodesAndEdges)
{
  PoseGraph g;
  g.addNode(0.0, 0.0, 0.0);
  g.addNode(1.0, 0.0, 0.0);
  g.addEdge(0, 1, 1.0, 0.0, 0.0);

  EXPECT_EQ(g.size(), 2);
  EXPECT_EQ(g.edgeCount(), 1);
  EXPECT_NEAR(g.totalError(), 0.0, 1e-12) << "位姿和观测完全一致，误差应为 0";
}

TEST(PoseGraph, FindLoopCandidatesByDistance)
{
  PoseGraph g;
  const double pts[5][3] = {
    {0.0, 0.0, 0.0}, {2.0, 0.0, 0.0}, {2.0, 2.0, 0.0},
    {0.0, 2.0, 0.0}, {0.0, 0.0, 0.0}};
  for (int i = 0; i < 5; ++i) {g.addNode(pts[i][0], pts[i][1], pts[i][2]);}

  const auto c0 = g.findLoopCandidates(4, 0.5, 2);
  EXPECT_EQ(c0.size(), 1u);
  EXPECT_EQ(c0[0], 0);

  const auto c1 = g.findLoopCandidates(4, 0.5, 5);
  EXPECT_TRUE(c1.empty());
}

TEST(PoseGraph, OptimizeClosesLoop)
{
  PoseGraph g;

  const double truth[5][3] = {
    {0.0, 0.0, 0.0}, {2.0, 0.0, 0.0}, {2.0, 2.0, 0.0},
    {0.0, 2.0, 0.0}, {0.0, 0.0, 0.0}};

  for (int i = 0; i < 5; ++i) {g.addNode(truth[i][0], truth[i][1], truth[i][2]);}

  // 里程计边：5% 尺度 + 每步 0.02 rad 偏转
  const double s = 1.05;
  const double dyaw_err = 0.02;
  g.addEdge(0, 1, 2.0 * s, 0.0, dyaw_err);
  g.addEdge(1, 2, 0.0, 2.0 * s, dyaw_err);
  g.addEdge(2, 3, -2.0 * s, 0.0, dyaw_err);
  g.addEdge(3, 4, 0.0, -2.0 * s, dyaw_err);

  // 用里程计重新推一遍位姿（模拟 SLAM 的初始状态）
  double x = 0.0, y = 0.0, yaw = 0.0;
  for (int i = 1; i < 5; ++i) {
    const double c = std::cos(yaw), sn = std::sin(yaw);
    const double dx = (i == 1) ? 2.0 * s : (i == 2 ? 0.0 : (i == 3 ? -2.0 * s : 0.0));
    const double dy = (i == 1) ? 0.0 : (i == 2 ? 2.0 * s : (i == 3 ? 0.0 : -2.0 * s));
    x += c * dx - sn * dy;
    y += sn * dx + c * dy;
    yaw += dyaw_err;
    g.setPose(i, x, y, yaw);
  }

  // 回环边：node4 观测到 node0，相对位姿 = (0,0,0)
  g.addEdge(4, 0, 0.0, 0.0, 0.0, 1.0, 1.0, true);

  const double err_before = g.totalError();
  const double n4_before = std::hypot(g.pose(4).x, g.pose(4).y);

  std::printf("  before: err=%.4f  node4=(%.4f, %.4f, %.4f)  dist_to_start=%.4f\n",
              err_before, g.pose(4).x, g.pose(4).y, g.pose(4).yaw, n4_before);

  const double err_after = g.optimize(500, 0.5);
  const double n4_after = std::hypot(g.pose(4).x, g.pose(4).y);

  std::printf("  after : err=%.4f  node4=(%.4f, %.4f, %.4f)  dist_to_start=%.4f\n",
              err_after, g.pose(4).x, g.pose(4).y, g.pose(4).yaw, n4_after);

  std::printf("  node errors (真值 vs 优化后):\n");
  for (int i = 0; i < 5; ++i) {
    const double ex = g.pose(i).x - truth[i][0];
    const double ey = g.pose(i).y - truth[i][1];
    std::printf("    node%d: (%.3f, %.3f) vs (%.3f, %.3f)  err=%.4f\n",
                i, g.pose(i).x, g.pose(i).y, truth[i][0], truth[i][1],
                std::hypot(ex, ey));
  }

  EXPECT_LT(err_after, err_before * 0.2) << "总误差应该明显下降";
  EXPECT_LT(n4_after, n4_before * 0.5) << "回环应该把 node4 拉回起点附近";
  EXPECT_LT(n4_after, 0.15) << "node4 到起点的距离应该很小";
}

TEST(PoseGraph, FixedNodeNeverMoves)
{
  PoseGraph g;
  g.addNode(0.0, 0.0, 0.0);
  g.addNode(1.0, 0.0, 0.0);
  g.addEdge(0, 1, 2.0, 0.0, 0.0);

  g.optimize(200, 0.5);

  EXPECT_NEAR(g.pose(0).x, 0.0, 1e-12) << "固定节点不能动";
  EXPECT_NEAR(g.pose(0).y, 0.0, 1e-12);
  EXPECT_NEAR(g.pose(1).x, 2.0, 0.01) << "可动节点应该被拉到观测位置";
}