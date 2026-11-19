#include "rm2027_roi/coding/crop_qpmap.hpp"

#include <algorithm>
#include <cmath>

namespace rm2027_roi {

Crop crop4x3(const RoiDetection& roi, int frame_w, int frame_h) {
  const double side = 4.0 * std::max(roi.w, 4.0 / 3.0 * roi.h);
  int wc = static_cast<int>(std::llround(
      std::max(static_cast<double>(kMinSide),
               std::min(static_cast<double>(kMaxSide), side))));
  wc = std::max(4, (wc + 3) / 4 * 4);
  const int hc = wc * 3 / 4;
  int x0 = static_cast<int>(std::llround(roi.cx - wc / 2.0));
  int y0 = static_cast<int>(std::llround(roi.cy - hc / 2.0));
  x0 = std::max(0, std::min(frame_w - wc, x0));
  y0 = std::max(0, std::min(frame_h - hc, y0));
  return Crop{x0, y0, wc, hc};
}

std::array<double, 4> roiOnCanvas(const RoiDetection& roi, const Crop& crop,
                                  int width, int height) {
  const double sx = static_cast<double>(width) / crop.w;
  const double sy = static_cast<double>(height) / crop.h;
  const double x = (roi.cx - roi.w / 2.0 - crop.x0) * sx;
  const double y = (roi.cy - roi.h / 2.0 - crop.y0) * sy;
  return {x, y, roi.w * sx, roi.h * sy};
}

Qpmap buildQpmap(const std::array<double, 4>& roi_canvas, int width,
                 int height, int bg_qp, int fg_qp) {
  Qpmap q;
  q.data.fill(static_cast<int8_t>(bg_qp));
  constexpr int kBlock = 16;
  const int mb_w = (width + kBlock - 1) / kBlock;
  const int mb_h = (height + kBlock - 1) / kBlock;
  const double x = roi_canvas[0];
  const double y = roi_canvas[1];
  const double w = roi_canvas[2];
  const double h = roi_canvas[3];
  const double x1 = x + w;
  const double y1 = y + h;
  for (int my = 0; my < mb_h; ++my) {
    const int by0 = my * kBlock;
    const int by1 = std::min(height, by0 + kBlock);
    if (by0 >= y1 || by1 <= y) {
      continue;
    }
    for (int mx = 0; mx < mb_w; ++mx) {
      const int bx0 = mx * kBlock;
      const int bx1 = std::min(width, bx0 + kBlock);
      if (bx0 < x1 && bx1 > x && by0 < y1 && by1 > y) {
        q.data[my * Qpmap::W + mx] = static_cast<int8_t>(fg_qp);
      }
    }
  }
  return q;
}

std::array<uint8_t, 4> normalizedRoi(const std::array<double, 4>& roi_canvas,
                                     int width, int height) {
  const double vals[4] = {
      roi_canvas[0] / static_cast<double>(width),
      roi_canvas[1] / static_cast<double>(height),
      roi_canvas[2] / static_cast<double>(width),
      roi_canvas[3] / static_cast<double>(height)};
  std::array<uint8_t, 4> out{};
  for (int i = 0; i < 4; ++i) {
    const double v = std::max(0.0, std::min(1.0, vals[i]));
    out[i] = static_cast<uint8_t>(std::llround(v * 255.0));
  }
  return out;
}

}  // namespace rm2027_roi
