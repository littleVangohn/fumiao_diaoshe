#pragma once

#include <deque>
#include <optional>
#include <vector>

#include <opencv2/core.hpp>

#include "rm2027_roi/core/types.hpp"
#include "rm2027_roi/core/parameters.hpp"

namespace rm2027_roi {

struct Pair {
  Bar a;
  Bar b;
  double sep = 0.0;
};

class RGDetector {
 public:
  explicit RGDetector(const RgParams& params = RgParams());

  RoiDetection detect(const cv::Mat& bgr);
  void reset();

  // Exposed for geometry tests / offline diagnostics.
  Lamp greenLamp(const cv::Mat& hsv) const;
  std::vector<Bar> redBars(const cv::Mat& hsv, const Lamp& lamp) const;
  std::optional<Pair> choosePair(const std::vector<Bar>& bars,
                                 const Lamp& lamp) const;
  RoiDetection buildPairRoi(const Lamp& lamp, const Pair& pair) const;
  RoiDetection lampRoi(const Lamp& lamp) const;

  int frameIndex() const { return frame_idx_; }
  std::size_t templateSize() const { return hist_.size(); }

 private:
  struct Sample {
    cv::Vec2d uL;
    cv::Vec2d uR;
    cv::Vec2d d;
    double ws = 0.0;
    double hs = 0.0;
    double barh = 0.0;
  };

  struct Stats {
    cv::Vec2d uL_mean;
    cv::Vec2d uR_mean;
    cv::Vec2d d_mean;
    cv::Matx22d uL_cov;
    cv::Matx22d uR_cov;
    double ws = 0.0;
    double hs = 0.0;
    double barh = 0.0;
  };

  void updateTemplate(const Lamp& lamp, const Pair& pair,
                      const RoiDetection& roi);
  std::optional<Stats> stats() const;
  bool templateValid() const;
  RoiDetection fallback(const Lamp& lamp, const std::vector<Bar>& bars);

  RgParams params_;
  std::deque<Sample> hist_;
  int frame_idx_ = -1;
  int last_pair_idx_ = -1000000000;
};

}  // namespace rm2027_roi
