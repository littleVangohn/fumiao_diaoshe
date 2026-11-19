#include "rm2027_roi/core/roi_pipeline.hpp"

#include <algorithm>

namespace rm2027_roi {

RoiPipeline::RoiPipeline(const KalmanParams& kalman, const RgParams& rg)
    : detector_(rg), estimator_(kalman) {}

void RoiPipeline::reset() {
  detector_.reset();
  estimator_.reset();
  last_roi_ = RoiDetection{};
}

RoiEstimate RoiPipeline::process(const cv::Mat& bgr, double ts,
                                 FrameResult* result) {
  RoiDetection det = detector_.detect(bgr);
  RoiEstimate est = estimator_.update(det, ts);

  RoiDetection smooth;
  smooth.valid = est.valid;
  smooth.cx = est.cx;
  smooth.cy = est.cy;
  smooth.w = est.w;
  smooth.h = est.h;
  smooth.method = est.source;
  smooth.confidence = est.uncertainty;
  if (smooth.valid) {
    last_roi_ = smooth;
  } else if (last_roi_.valid) {
    smooth = last_roi_;
  } else if (det.valid) {
    smooth = det;
    last_roi_ = det;
  } else {
    smooth.valid = true;
    smooth.cx = bgr.cols / 2.0;
    smooth.cy = bgr.rows / 2.0;
    smooth.w = 200.0;
    smooth.h = 100.0;
    smooth.method = "FALLBACK";
    smooth.confidence = 0.0;
    last_roi_ = smooth;
  }

  if (result) {
    result->detection = det;
    result->estimate = est;
    result->valid_crop = smooth.valid && smooth.w > 0.0 && smooth.h > 0.0;
    if (result->valid_crop) {
      result->crop = crop4x3(smooth, bgr.cols, bgr.rows);
      result->roi_on_canvas = roiOnCanvas(smooth, result->crop);
      result->qmap = buildQpmap(result->roi_on_canvas);
      result->roi_normalized = normalizedRoi(result->roi_on_canvas);
    }
  }
  return est;
}

}  // namespace rm2027_roi
