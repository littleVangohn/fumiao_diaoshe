#pragma once

#include <array>
#include <opencv2/core.hpp>

#include "rm2027_roi/coding/crop_qpmap.hpp"
#include "rm2027_roi/estimation/kalman_estimator.hpp"
#include "rm2027_roi/detection/rg_detector.hpp"
#include "rm2027_roi/core/types.hpp"
#include "rm2027_roi/core/parameters.hpp"

namespace rm2027_roi {

struct FrameResult {
  RoiDetection detection;
  RoiEstimate estimate;
  Crop crop;
  std::array<double, 4> roi_on_canvas{};
  Qpmap qmap;
  std::array<uint8_t, 4> roi_normalized{};
  bool valid_crop = false;
};

class RoiPipeline {
 public:
  RoiPipeline(const KalmanParams& kalman = KalmanParams(),
              const RgParams& rg = RgParams());

  RoiEstimate process(const cv::Mat& bgr, double ts, FrameResult* result);
  void reset();

 private:
  RGDetector detector_;
  KalmanEstimator estimator_;
  RoiDetection last_roi_;
};

}  // namespace rm2027_roi
