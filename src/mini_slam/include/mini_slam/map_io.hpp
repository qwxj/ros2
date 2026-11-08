#ifndef MINI_SLAM__MAP_IO_HPP_
#define MINI_SLAM__MAP_IO_HPP_

#include <cstdint>
#include <string>
#include <vector>

#include "mini_slam/occupancy_grid.hpp"

namespace mini_slam
{

struct MapMetadata
{
  std::string image{"map.pgm"};
  std::string mode{"trinary"};
  double resolution{0.05};
  double origin_x{0.0};
  double origin_y{0.0};
  double origin_yaw{0.0};
  int negate{0};
  double occupied_thresh{0.65};
  double free_thresh{0.25};
};

/// 像素值 → 占据概率（0~1），完全按 nav2_map_server 的规则
double pixelToOccupancy(int pixel, int negate);

/// 占据概率 → ROS 三值（100 / 0 / -1）
int8_t occupancyToValue(double occ, double occupied_thresh, double free_thresh);

/// ROS 三值 → PGM 像素（100→0，0→254，-1→205）
int valueToPixel(int8_t value);

/// 读 PGM（P5 二进制）
bool readPgm(const std::string & path, int & w, int & h,
             std::vector<uint8_t> & pixels, std::string * err = nullptr);

/// 写 PGM（P5 二进制）
bool writePgm(const std::string & path, int w, int h,
              const std::vector<uint8_t> & pixels, std::string * err = nullptr);

/// 读 yaml + pgm → OccupancyGrid
bool loadMap(const std::string & yaml_path, OccupancyGrid & out,
             MapMetadata & meta, std::string * err = nullptr);

/// OccupancyGrid → yaml + pgm
bool saveMap(const std::string & yaml_path, const OccupancyGrid & grid,
             const MapMetadata & meta, std::string * err = nullptr);

}  // namespace mini_slam

#endif