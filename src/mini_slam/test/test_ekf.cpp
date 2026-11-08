#include "mini_slam/ekf.hpp"

#include <gtest/gtest.h>

#include <cmath>

using mini_slam::Ekf2D;

TEST(Ekf2D, Initialize)
{
  Ekf2D ekf;
  EXPECT_FALSE(ekf.initialized());

  ekf.initialize(1.0, 2.0, 0.5);
  EXPECT_TRUE(ekf.initialized());
  EXPECT_NEAR(ekf.x(), 1.0, 1e-12);
  EXPECT_NEAR(ekf.y(), 2.0, 1e-12);
  EXPECT_NEAR(ekf.yaw(), 0.5, 1e-12);
}

TEST(Ekf2D, PredictStraightLine)
{
  Ekf2D ekf;
  ekf.initialize(0.0, 0.0, 0.0);
  ekf.updateOdomTwist(1.0, 0.0, 0.01, 0.01);

  // 100 步 × 0.01 s = 1 s，vx = 1 m/s → 应该走 1 m
  for (int i = 0; i < 100; ++i) {ekf.predict(0.01, 1e-4, 1e-4);}

  EXPECT_NEAR(ekf.x(), 1.0, 0.05);
  EXPECT_NEAR(ekf.y(), 0.0, 0.05);
  EXPECT_NEAR(ekf.yaw(), 0.0, 0.02);
}

TEST(Ekf2D, PredictCircularArc)
{
  Ekf2D ekf;
  ekf.initialize(0.0, 0.0, 0.0);
  ekf.updateOdomTwist(1.0, 1.0, 0.01, 0.01);   // R = v/w = 1.0

  for (int i = 0; i < 157; ++i) {ekf.predict(0.01, 1e-4, 1e-4);}

  // 转了 θ = 1.57 rad ≈ 90°，走过 1/4 圆
  // 终点到原点的【弦长】= 2R*sin(θ/2) = 2*sin(0.785) = √2
  const double r = std::hypot(ekf.x(), ekf.y());
  EXPECT_NEAR(r, std::sqrt(2.0), 0.05) << "弦长是 sqrt(2)，不是半径 1.0";
  EXPECT_NEAR(ekf.x(), 1.0, 0.05);
  EXPECT_NEAR(ekf.y(), 1.0, 0.05);
  EXPECT_NEAR(ekf.yaw(), M_PI / 2.0, 0.05);
}

TEST(Ekf2D, UpdateYawRatePullsState)
{
  Ekf2D ekf;
  ekf.initialize(0.0, 0.0, 0.0);
  EXPECT_NEAR(ekf.vyaw(), 0.0, 1e-12);

  for (int i = 0; i < 50; ++i) {ekf.updateYawRate(0.8, 0.05);}
  EXPECT_NEAR(ekf.vyaw(), 0.8, 0.01);
}

TEST(Ekf2D, UpdatePosePullsPosition)
{
  Ekf2D ekf;
  ekf.initialize(0.0, 0.0, 0.0);

  for (int i = 0; i < 50; ++i) {ekf.updatePose(2.0, -1.0, 0.3, 0.1, 0.02);}

  EXPECT_NEAR(ekf.x(), 2.0, 0.01);
  EXPECT_NEAR(ekf.y(), -1.0, 0.01);
  EXPECT_NEAR(ekf.yaw(), 0.3, 0.01);
}

TEST(Ekf2D, CovarianceGrowsOnPredictShrinksOnUpdate)
{
  Ekf2D ekf;
  ekf.initialize(0.0, 0.0, 0.0);
  ekf.updateOdomTwist(1.0, 0.1, 0.01, 0.01);

  const double t0 = ekf.covariance().trace();
  for (int i = 0; i < 50; ++i) {ekf.predict(0.01, 1e-3, 1e-3);}
  const double t1 = ekf.covariance().trace();
  EXPECT_GT(t1, t0) << "预测应该让不确定性变大";

  for (int i = 0; i < 20; ++i) {ekf.updatePose(0.0, 0.0, 0.0, 0.02, 0.01);}
  const double t2 = ekf.covariance().trace();
  EXPECT_LT(t2, t1) << "更新应该让不确定性变小";
}

TEST(Ekf2D, YawWrapsAcrossPi)
{
  Ekf2D ekf;
  ekf.initialize(0.0, 0.0, 3.0);
  ekf.updateOdomTwist(0.0, 1.0, 0.01, 0.01);

  for (int i = 0; i < 100; ++i) {ekf.predict(0.01, 1e-4, 1e-4);}

  EXPECT_GE(ekf.yaw(), -M_PI - 1e-9);
  EXPECT_LE(ekf.yaw(), M_PI + 1e-9);
}