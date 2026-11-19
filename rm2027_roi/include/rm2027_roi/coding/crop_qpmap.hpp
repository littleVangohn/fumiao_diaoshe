#pragma once

#include <array>
#include <cstdint>

#include "rm2027_roi/core/types.hpp"

namespace rm2027_roi {

constexpr int kCanvasW = 320;
constexpr int kCanvasH = 240;
constexpr int kMinSide = 192;
constexpr int kMaxSide = 400;
constexpr int kBgQp = 10;
constexpr int kFgQp = -5;

Crop crop4x3(const RoiDetection& roi, int frame_w, int frame_h);
std::array<double, 4> roiOnCanvas(const RoiDetection& roi, const Crop& crop,
                                  int width = kCanvasW,
                                  int height = kCanvasH);
Qpmap buildQpmap(const std::array<double, 4>& roi_canvas,
                 int width = kCanvasW, int height = kCanvasH,
                 int bg_qp = kBgQp, int fg_qp = kFgQp);
std::array<uint8_t, 4> normalizedRoi(const std::array<double, 4>& roi_canvas,
                                     int width = kCanvasW,
                                     int height = kCanvasH);

}  // namespace rm2027_roi
