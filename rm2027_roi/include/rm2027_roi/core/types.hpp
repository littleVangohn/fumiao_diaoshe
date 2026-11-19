#pragma once

#include <array>
#include <cstdint>
#include <string>

namespace rm2027_roi {

struct Lamp {
  double cx = 0.0;
  double cy = 0.0;
  int x = 0;
  int y = 0;
  int w = 0;
  int h = 0;
  double scale = 0.0;
};

struct Bar {
  int x = 0;
  int y = 0;
  int w = 0;
  int h = 0;
  double cx = 0.0;
  double cy = 0.0;
  double tilt = 0.0;
};

struct RoiDetection {
  bool valid = false;
  double cx = 0.0;
  double cy = 0.0;
  double w = 0.0;
  double h = 0.0;
  std::string method = "NONE";
  double confidence = 0.0;
};

struct RoiEstimate {
  bool valid = false;
  double cx = 0.0;
  double cy = 0.0;
  double w = 0.0;
  double h = 0.0;
  std::string state = "LOST";
  std::string source = "NONE";
  double horizon = 0.0;
  double uncertainty = 0.0;
  double cost_ms = 0.0;
};

struct Crop {
  int x0 = 0;
  int y0 = 0;
  int w = 0;
  int h = 0;
};

struct Qpmap {
  static constexpr int W = 20;
  static constexpr int H = 15;
  std::array<int8_t, W * H> data{};
};

}  // namespace rm2027_roi
