#include "mini_slam/map_io.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iterator>
#include <string>
#include <vector>

namespace mini_slam
{

double pixelToOccupancy(int pixel, int negate)
{
  // nav2_map_server 的规则：
  //   negate = 0  →  occ = (255 - pixel) / 255   （黑 = 占据）
  //   negate = 1  →  occ = pixel / 255           （白 = 占据）
  const double v = static_cast<double>(pixel) / 255.0;
  return negate ? v : (1.0 - v);
}

int8_t occupancyToValue(double occ, double occupied_thresh, double free_thresh)
{
  if (occ > occupied_thresh) {return 100;}
  if (occ < free_thresh) {return 0;}
  return -1;
}

int valueToPixel(int8_t value)
{
  if (value < 0) {return 205;}      // 未知：经典灰
  if (value >= 50) {return 0;}      // 占据：黑
  return 254;                        // 空闲：白
}

// ------------------------------------------------------------------
// PGM 头部解析：支持 P2（ASCII）和 P5（二进制）
//   格式："P2/P5 <w> <h> <maxval> <data>"，允许 # 注释
// ------------------------------------------------------------------
namespace
{

struct PgmHeader
{
  bool ascii{false};
  int w{0};
  int h{0};
  int maxval{255};
  size_t data_offset{0};
};

bool parsePgmHeader(const std::vector<char> & buf, PgmHeader & hdr, std::string * err)
{
  size_t i = 0;

  auto skip_ws_and_comments = [&]() {
      while (i < buf.size()) {
        const char c = buf[i];
        if (c == '#') {
          while (i < buf.size() && buf[i] != '\n') {++i;}
        } else if (c == ' ' || c == '\t' || c == '\r' || c == '\n') {
          ++i;
        } else {
          break;
        }
      }
    };

  auto next_int = [&](int & out) -> bool {
      skip_ws_and_comments();
      if (i >= buf.size()) {return false;}
      int v = 0;
      bool any = false;
      while (i < buf.size() && buf[i] >= '0' && buf[i] <= '9') {
        v = v * 10 + (buf[i] - '0');
        ++i;
        any = true;
      }
      if (!any) {return false;}
      out = v;
      return true;
    };

  skip_ws_and_comments();
  if (i + 1 >= buf.size() || buf[i] != 'P' || (buf[i + 1] != '5' && buf[i + 1] != '2')) {
    if (err) {*err = "not a pgm (only P2/P5 supported)";}
    return false;
  }
  hdr.ascii = (buf[i + 1] == '2');
  i += 2;

  if (!next_int(hdr.w) || !next_int(hdr.h) || !next_int(hdr.maxval)) {
    if (err) {*err = "bad pgm header";}
    return false;
  }
  if (i >= buf.size()) {
    if (err) {*err = "truncated pgm header";}
    return false;
  }
  ++i;                        // 头部与数据之间正好一个空白字符
  hdr.data_offset = i;
  return true;
}

}  // namespace

bool readPgm(const std::string & path, int & w, int & h,
             std::vector<uint8_t> & pixels, std::string * err)
{
  std::ifstream in(path, std::ios::binary);
  if (!in) {
    if (err) {*err = "cannot open: " + path;}
    return false;
  }
  std::vector<char> buf((std::istreambuf_iterator<char>(in)),
                        std::istreambuf_iterator<char>());

  PgmHeader hdr;
  if (!parsePgmHeader(buf, hdr, err)) {return false;}
  if (hdr.maxval != 255) {
    if (err) {*err = "only maxval=255 supported";}
    return false;
  }

  const size_t need = static_cast<size_t>(hdr.w) * hdr.h;
  w = hdr.w;
  h = hdr.h;
  pixels.assign(need, 0);

  if (!hdr.ascii) {
    // ---------- P5：直接拷贝二进制 ----------
    if (buf.size() < hdr.data_offset + need) {
      if (err) {*err = "truncated pixel data";}
      return false;
    }
    for (size_t k = 0; k < need; ++k) {
      pixels[k] = static_cast<uint8_t>(buf[hdr.data_offset + k]);
    }
    return true;
  }

  // ---------- P2：解析 ASCII 整数 ----------
  size_t p = hdr.data_offset;
  size_t k = 0;
  while (k < need) {
    while (p < buf.size()) {
      const char c = buf[p];
      if (c == '#') {
        while (p < buf.size() && buf[p] != '\n') {++p;}
      } else if (c == ' ' || c == '\t' || c == '\r' || c == '\n') {
        ++p;
      } else {
        break;
      }
    }
    if (p >= buf.size()) {break;}
    int v = 0;
    bool any = false;
    while (p < buf.size() && buf[p] >= '0' && buf[p] <= '9') {
      v = v * 10 + (buf[p] - '0');
      ++p;
      any = true;
    }
    if (!any) {++p; continue;}
    pixels[k++] = static_cast<uint8_t>(std::min(255, v));
  }

  if (k != need) {
    if (err) {*err = "truncated ascii pixel data";}
    return false;
  }
  return true;
}

bool writePgm(const std::string & path, int w, int h,
              const std::vector<uint8_t> & pixels, std::string * err)
{
  if (static_cast<size_t>(w) * h != pixels.size()) {
    if (err) {*err = "pixel count mismatch";}
    return false;
  }
  std::ofstream out(path, std::ios::binary);
  if (!out) {
    if (err) {*err = "cannot write: " + path;}
    return false;
  }
  out << "P5\n" << w << " " << h << "\n255\n";
  out.write(reinterpret_cast<const char *>(pixels.data()),
            static_cast<std::streamsize>(pixels.size()));
  return static_cast<bool>(out);
}

// ------------------------------------------------------------------
// 读地图
// ------------------------------------------------------------------
bool loadMap(const std::string & yaml_path, OccupancyGrid & out,
             MapMetadata & meta, std::string * err)
{
  YAML::Node root;
  try {
    root = YAML::LoadFile(yaml_path);
  } catch (const std::exception & e) {
    if (err) {*err = std::string("yaml parse error: ") + e.what();}
    return false;
  }

  if (root["image"]) {meta.image = root["image"].as<std::string>();}
  if (root["mode"]) {meta.mode = root["mode"].as<std::string>();}
  if (root["resolution"]) {meta.resolution = root["resolution"].as<double>();}
  if (root["negate"]) {meta.negate = root["negate"].as<int>();}
  if (root["occupied_thresh"]) {meta.occupied_thresh = root["occupied_thresh"].as<double>();}
  if (root["free_thresh"]) {meta.free_thresh = root["free_thresh"].as<double>();}

  if (root["origin"] && root["origin"].size() >= 2) {
    meta.origin_x = root["origin"][0].as<double>();
    meta.origin_y = root["origin"][1].as<double>();
    if (root["origin"].size() >= 3) {meta.origin_yaw = root["origin"][2].as<double>();}
  }

  // 图片路径：相对 yaml 所在目录
  std::string img_path = meta.image;
  if (img_path.empty() || (img_path[0] != '/')) {
    const size_t slash = yaml_path.find_last_of('/');
    if (slash != std::string::npos) {
      img_path = yaml_path.substr(0, slash + 1) + img_path;
    }
  }

  int w = 0, h = 0;
  std::vector<uint8_t> px;
  if (!readPgm(img_path, w, h, px, err)) {return false;}

  out = OccupancyGrid(meta.resolution, meta.origin_x, meta.origin_y, w, h);

  // PGM 是【从上到下】存，而栅格是【y 递增向上】存 → 要翻转 y
  for (int row = 0; row < h; ++row) {
    const int gy = h - 1 - row;
    for (int gx = 0; gx < w; ++gx) {
      const int pixel = px[static_cast<size_t>(row) * w + gx];
      const double occ = pixelToOccupancy(pixel, meta.negate);
      out.setOccupancyValue(gx, gy,
                            occupancyToValue(occ, meta.occupied_thresh, meta.free_thresh));
    }
  }
  return true;
}

// ------------------------------------------------------------------
// 写地图
// ------------------------------------------------------------------
bool saveMap(const std::string & yaml_path, const OccupancyGrid & grid,
             const MapMetadata & meta, std::string * err)
{
  const int w = grid.width();
  const int h = grid.height();
  std::vector<uint8_t> px(static_cast<size_t>(w) * h, 205);

  for (int gy = 0; gy < h; ++gy) {
    const int row = h - 1 - gy;
    for (int gx = 0; gx < w; ++gx) {
      int8_t value = -1;
      if (grid.observed(gx, gy)) {
        value = (grid.logOdds(gx, gy) > 0.0) ? 100 : 0;
      }
      px[static_cast<size_t>(row) * w + gx] = static_cast<uint8_t>(valueToPixel(value));
    }
  }

  std::string img_path = meta.image;
  const size_t slash = yaml_path.find_last_of('/');
  if (slash != std::string::npos && (img_path.empty() || img_path[0] != '/')) {
    img_path = yaml_path.substr(0, slash + 1) + img_path;
  }

  if (!writePgm(img_path, w, h, px, err)) {return false;}

  std::ofstream out(yaml_path);
  if (!out) {
    if (err) {*err = "cannot write: " + yaml_path;}
    return false;
  }
  out.setf(std::ios::fixed);
  out.precision(6);
  out << "image: " << meta.image << "\n";
  out << "mode: " << meta.mode << "\n";
  out << "resolution: " << meta.resolution << "\n";
  out << "origin: ["
      << meta.origin_x << ", " << meta.origin_y << ", " << meta.origin_yaw << "]\n";
  out << "negate: " << meta.negate << "\n";
  out << "occupied_thresh: " << meta.occupied_thresh << "\n";
  out << "free_thresh: " << meta.free_thresh << "\n";
  return true;
}

}  // namespace mini_slam
