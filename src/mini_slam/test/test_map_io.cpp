#include "mini_slam/map_io.hpp"
#include "mini_slam/occupancy_grid.hpp"

#include <gtest/gtest.h>

#include <cstdio>
#include <fstream>
#include <map>
#include <string>
#include <vector>

using mini_slam::MapMetadata;
using mini_slam::OccupancyGrid;

static const std::string kRealYaml =
  "/home/wxj/ros2/src/my_robot_description/maps/my_map.yaml";

static std::map<int, int> countValues(const OccupancyGrid & g)
{
  std::map<int, int> c;
  for (int gy = 0; gy < g.height(); ++gy) {
    for (int gx = 0; gx < g.width(); ++gx) {
      const int8_t v = g.probability(gx, gy) > 50 ? 100
        : (g.observed(gx, gy) ? 0 : -1);
      ++c[v];
    }
  }
  return c;
}

static void printCounts(const std::map<int, int> & c, const char * tag)
{
  int total = 0;
  for (const auto & kv : c) {total += kv.second;}
  std::printf("  %s: total=%d\n", tag, total);
  for (const auto & kv : c) {
    std::printf("    value %4d : %6d (%.1f%%)\n",
                kv.first, kv.second, 100.0 * kv.second / total);
  }
}

TEST(MapIo, PixelToOccupancyRules)
{
  using mini_slam::pixelToOccupancy;
  using mini_slam::occupancyToValue;

  // negate = 0：黑 = 占据
  EXPECT_NEAR(pixelToOccupancy(0, 0), 1.000, 1e-9);
  EXPECT_NEAR(pixelToOccupancy(254, 0), 1.0 / 255.0, 1e-9);
  EXPECT_NEAR(pixelToOccupancy(205, 0), 50.0 / 255.0, 1e-9);

  // negate = 1：白 = 占据
  EXPECT_NEAR(pixelToOccupancy(0, 1), 0.0, 1e-9);
  EXPECT_NEAR(pixelToOccupancy(255, 1), 1.0, 1e-9);

  // 三值判定
  EXPECT_EQ(occupancyToValue(1.000, 0.65, 0.25), 100);
  EXPECT_EQ(occupancyToValue(0.004, 0.65, 0.25), 0);
  EXPECT_EQ(occupancyToValue(0.500, 0.65, 0.25), -1);

  // ★ 关键：205 灰像素在 free_thresh=0.25 下是【空闲】，在 0.15 下才是【未知】
  EXPECT_EQ(occupancyToValue(pixelToOccupancy(205, 0), 0.65, 0.25), 0);
  EXPECT_EQ(occupancyToValue(pixelToOccupancy(205, 0), 0.65, 0.15), -1);
}

TEST(MapIo, PgmRoundTrip)
{
  const std::string path = "/tmp/mio_test_rt.pgm";
  const int w = 7, h = 5;
  std::vector<uint8_t> px(static_cast<size_t>(w) * h);
  for (size_t i = 0; i < px.size(); ++i) {px[i] = static_cast<uint8_t>(i * 3 % 256);}

  std::string err;
  ASSERT_TRUE(mini_slam::writePgm(path, w, h, px, &err)) << err;

  int w2 = 0, h2 = 0;
  std::vector<uint8_t> px2;
  ASSERT_TRUE(mini_slam::readPgm(path, w2, h2, px2, &err)) << err;

  EXPECT_EQ(w2, w);
  EXPECT_EQ(h2, h);
  ASSERT_EQ(px2.size(), px.size());
  for (size_t i = 0; i < px.size(); ++i) {EXPECT_EQ(px2[i], px[i]) << "at " << i;}
  std::remove(path.c_str());
}

TEST(MapIo, LoadRealMapMatchesMapServer)
{
  std::ifstream probe(kRealYaml);
  if (!probe) {GTEST_SKIP() << "真实地图不存在: " << kRealYaml;}

  OccupancyGrid g(0.05, 0, 0, 1, 1);
  MapMetadata meta;
  std::string err;
  ASSERT_TRUE(mini_slam::loadMap(kRealYaml, g, meta, &err)) << err;

  EXPECT_EQ(g.width(), 110);
  EXPECT_EQ(g.height(), 81);
  EXPECT_NEAR(meta.resolution, 0.05, 1e-9);
  EXPECT_NEAR(meta.origin_x, -1.99, 1e-9);
  EXPECT_NEAR(meta.origin_y, -2.04, 1e-9);
  EXPECT_NEAR(meta.free_thresh, 0.25, 1e-9);

  const auto c = countValues(g);
  printCounts(c, "real map");

  EXPECT_EQ(c.at(0), 8698) << "空闲格子数必须和真实 map_server 一致";
  EXPECT_EQ(c.at(100), 212) << "占据格子数必须和真实 map_server 一致";
  EXPECT_EQ(c.count(-1) ? c.at(-1) : 0, 0) << "free_thresh=0.25 时灰像素被算成空闲";
}

TEST(MapIo, RealMapRoundTripIsStable)
{
  std::ifstream probe(kRealYaml);
  if (!probe) {GTEST_SKIP() << "真实地图不存在";}

  OccupancyGrid g1(0.05, 0, 0, 1, 1);
  MapMetadata meta;
  std::string err;
  ASSERT_TRUE(mini_slam::loadMap(kRealYaml, g1, meta, &err)) << err;
  const auto c1 = countValues(g1);

  const std::string out_yaml = "/tmp/mio_roundtrip.yaml";
  meta.image = "mio_roundtrip.pgm";
  ASSERT_TRUE(mini_slam::saveMap(out_yaml, g1, meta, &err)) << err;

  {
    std::ifstream f(out_yaml);
    std::string line;
    std::printf("  written yaml:\n");
    while (std::getline(f, line)) {std::printf("    %s\n", line.c_str());}
  }

  OccupancyGrid g2(0.05, 0, 0, 1, 1);
  MapMetadata meta2;
  ASSERT_TRUE(mini_slam::loadMap(out_yaml, g2, meta2, &err)) << err;
  const auto c2 = countValues(g2);

  EXPECT_EQ(c1.at(0), c2.at(0));
  EXPECT_EQ(c1.at(100), c2.at(100));
  EXPECT_EQ(g2.width(), g1.width());
  EXPECT_EQ(g2.height(), g1.height());
}

TEST(MapIo, LowerFreeThreshTurnsGrayIntoUnknown)
{
  std::ifstream probe(kRealYaml);
  if (!probe) {GTEST_SKIP() << "真实地图不存在";}

  OccupancyGrid g0(0.05, 0, 0, 1, 1);
  MapMetadata meta;
  std::string err;
  ASSERT_TRUE(mini_slam::loadMap(kRealYaml, g0, meta, &err)) << err;

  const std::string tmp_yaml = "/tmp/mio_lowfree.yaml";
  {
    std::ofstream f(tmp_yaml);
    f << "image: " << kRealYaml.substr(0, kRealYaml.find_last_of('/') + 1) << "my_map.pgm\n";
    f << "mode: trinary\n";
    f << "resolution: " << meta.resolution << "\n";
    f << "origin: [" << meta.origin_x << ", " << meta.origin_y << ", 0]\n";
    f << "negate: 0\n";
    f << "occupied_thresh: " << meta.occupied_thresh << "\n";
    f << "free_thresh: 0.15\n";
  }

  OccupancyGrid g(0.05, 0, 0, 1, 1);
  MapMetadata m2;
  ASSERT_TRUE(mini_slam::loadMap(tmp_yaml, g, m2, &err)) << err;
  const auto c = countValues(g);
  printCounts(c, "real map, free_thresh=0.15");

  EXPECT_EQ(c.at(100), 212);
  ASSERT_TRUE(c.count(-1)) << "应该有未知格子了";
  EXPECT_EQ(c.at(-1), 1846) << "灰色像素 205 全部变成未知";
}

// ============================================================
// 6. 读 P2（ASCII）格式的地图：my_robot_control 的 map.pgm
//    P2 和 P5 都要支持 —— 前者是文本，后者是二进制，
//    但 map_server 两者都吃，所以 map_io 也必须都吃
// ============================================================
TEST(MapIo, LoadP2AsciiMap)
{
  const std::string yaml = "/home/wxj/ros2/src/my_robot_control/maps/map.yaml";
  std::ifstream probe(yaml);
  if (!probe) {GTEST_SKIP() << "P2 地图不存在: " << yaml;}

  OccupancyGrid g(0.05, 0, 0, 1, 1);
  MapMetadata meta;
  std::string err;
  ASSERT_TRUE(mini_slam::loadMap(yaml, g, meta, &err)) << err;

  EXPECT_EQ(g.width(), 200);
  EXPECT_EQ(g.height(), 160);
  EXPECT_NEAR(meta.origin_x, -5.0, 1e-9);
  EXPECT_NEAR(meta.origin_y, -4.0, 1e-9);
  EXPECT_NEAR(meta.free_thresh, 0.196, 1e-9);

  const auto c = countValues(g);
  printCounts(c, "P2 map (my_robot_control)");
  EXPECT_GT(c.at(100), 100) << "应该有很多占据格";
}
