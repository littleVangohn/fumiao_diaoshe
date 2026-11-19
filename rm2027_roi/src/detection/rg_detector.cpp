#include "rm2027_roi/detection/rg_detector.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include <opencv2/imgproc.hpp>

namespace rm2027_roi {
namespace {

constexpr int kGreenHLo = 35, kGreenHHi = 100;
constexpr int kGreenSMin = 40, kGreenVMin = 80;
constexpr int kGreenMinPix = 8;
constexpr double kGreenMaxFrac = 0.06;
constexpr double kGreenAspectMax = 3.5;
constexpr double kGreenFillMin = 0.08;

constexpr int kRedHLo = 12, kRedHHi = 165;
constexpr int kRedSMin = 55, kRedVMin = 60;
constexpr int kRedMinPix = 6;
constexpr double kRedSearchBelowRel = 8.0;

constexpr double kBarAspectMin = 1.4;
constexpr double kBarLenMinRel = 0.12;
constexpr double kBarLenMaxRel = 1.0;
constexpr double kBarBelowMinRel = 0.40;
constexpr double kBarXMaxRel = 3.5;
constexpr double kBarTiltMaxDeg = 75.0;

constexpr double kPairSepMinRel = 0.40;
constexpr double kPairSepMaxRel = 2.60;
constexpr double kPairSepPrefRel = 1.20;
constexpr double kPairHsimMin = 0.20;
constexpr double kPairYdifMaxRel = 0.75;
constexpr double kPairTiltDifMax = 22.0;
constexpr double kPairMidxMaxRel = 1.20;
constexpr double kPairMidyMinRel = 0.60;

constexpr double kPlateWOverSep = 1.09;
constexpr double kPlateHOverBarH = 2.18;
constexpr double kRoiCyShiftOverSep = 0.05;
constexpr double kLampRoiWRel = 1.30;
constexpr double kLampRoiHRel = 0.68;
constexpr double kLampRoiDyRel = 1.56;

const cv::Mat& k3x3() {
  static const cv::Mat kernel = cv::getStructuringElement(
      cv::MORPH_ELLIPSE, cv::Size(3, 3));
  return kernel;
}

std::optional<std::vector<cv::Point>> componentContour(
    const cv::Mat& labels, int index, const cv::Rect& bbox) {
  cv::Mat sub = (labels(bbox) == index);
  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(sub, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
  if (contours.empty()) {
    return std::nullopt;
  }
  auto best = std::max_element(
      contours.begin(), contours.end(),
      [](const std::vector<cv::Point>& a, const std::vector<cv::Point>& b) {
        return cv::contourArea(a) < cv::contourArea(b);
      });
  std::vector<cv::Point> out = *best;
  for (auto& p : out) {
    p.x += bbox.x;
    p.y += bbox.y;
  }
  return out;
}

double tiltFromVertical(const std::vector<cv::Point>& contour) {
  cv::Vec4f line;
  cv::fitLine(contour, line, cv::DIST_L2, 0, 0.01, 0.01);
  return std::atan2(std::abs(line[0]), std::abs(line[1])) * 180.0 / CV_PI;
}

cv::Matx22d inverse2(const cv::Matx22d& m) {
  const double det = m(0, 0) * m(1, 1) - m(0, 1) * m(1, 0);
  const double inv_det = 1.0 / std::max(std::abs(det), 1e-12);
  return cv::Matx22d(m(1, 1) * inv_det, -m(0, 1) * inv_det,
                     -m(1, 0) * inv_det, m(0, 0) * inv_det);
}

}  // namespace

RGDetector::RGDetector(const RgParams& params) : params_(params) {}

void RGDetector::reset() {
  hist_.clear();
  frame_idx_ = -1;
  last_pair_idx_ = -1000000000;
}

Lamp RGDetector::greenLamp(const cv::Mat& hsv) const {
  cv::Mat mask;
  cv::inRange(hsv, cv::Scalar(kGreenHLo, kGreenSMin, kGreenVMin),
              cv::Scalar(kGreenHHi, 255, 255), mask);
  cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, k3x3());

  cv::Mat labels, stats, centroids;
  const int n = cv::connectedComponentsWithStats(mask, labels, stats,
                                                 centroids, 8, CV_32S);
  Lamp best;
  double best_area = -1.0;
  const double total = static_cast<double>(mask.total());
  for (int i = 1; i < n; ++i) {
    const int x = stats.at<int>(i, cv::CC_STAT_LEFT);
    const int y = stats.at<int>(i, cv::CC_STAT_TOP);
    const int w = stats.at<int>(i, cv::CC_STAT_WIDTH);
    const int h = stats.at<int>(i, cv::CC_STAT_HEIGHT);
    const int area = stats.at<int>(i, cv::CC_STAT_AREA);
    if (area < kGreenMinPix || area > kGreenMaxFrac * total) {
      continue;
    }
    if (std::max(w, h) > kGreenAspectMax * std::max(1, std::min(w, h))) {
      continue;
    }
    if (area < kGreenFillMin * w * h) {
      continue;
    }
    const auto contour = componentContour(labels, i, cv::Rect(x, y, w, h));
    if (!contour || contour->size() < 5 ||
        cv::contourArea(*contour) < kGreenMinPix) {
      continue;
    }
    const cv::RotatedRect ellipse = cv::fitEllipse(*contour);
    const double ew = ellipse.size.width;
    const double eh = ellipse.size.height;
    if (std::max(ew, eh) / std::max(1.0, std::min(ew, eh)) >
        kGreenAspectMax) {
      continue;
    }
    const cv::Moments moments = cv::moments(*contour);
    if (moments.m00 == 0.0) {
      continue;
    }
    if (area > best_area) {
      best_area = area;
      best.cx = moments.m10 / moments.m00;
      best.cy = moments.m01 / moments.m00;
      best.x = x;
      best.y = y;
      best.w = w;
      best.h = h;
      best.scale = static_cast<double>(std::max(w, h));
    }
  }
  return best;
}

std::vector<Bar> RGDetector::redBars(const cv::Mat& hsv,
                                     const Lamp& lamp) const {
  std::vector<Bar> out;
  if (lamp.scale <= 0.0) {
    return out;
  }
  const double lcx = lamp.cx;
  const double lcy = lamp.cy;
  const double lscale = lamp.scale;
  const int x0 = static_cast<int>(
      std::max(0.0, lcx - kBarXMaxRel * lscale - 2.0));
  const int x1 = static_cast<int>(
      std::min(static_cast<double>(hsv.cols),
               lcx + kBarXMaxRel * lscale + 3.0));
  const int y0 = static_cast<int>(std::max(0.0, lcy));
  const int y1 = static_cast<int>(
      std::min(static_cast<double>(hsv.rows),
               lcy + kRedSearchBelowRel * lscale + 3.0));
  if (x1 - x0 < 4 || y1 - y0 < 4) {
    return out;
  }
  const cv::Rect roi_rect(x0, y0, x1 - x0, y1 - y0);
  const cv::Mat roi = hsv(roi_rect);
  cv::Mat red_lo, red_hi;
  cv::inRange(roi, cv::Scalar(0, kRedSMin, kRedVMin),
              cv::Scalar(kRedHLo, 255, 255), red_lo);
  cv::inRange(roi, cv::Scalar(kRedHHi, kRedSMin, kRedVMin),
              cv::Scalar(179, 255, 255), red_hi);
  cv::Mat mask = red_lo | red_hi;
  cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, k3x3());

  cv::Mat labels, stats, centroids;
  const int n = cv::connectedComponentsWithStats(mask, labels, stats,
                                                 centroids, 8, CV_32S);
  for (int i = 1; i < n; ++i) {
    const int x = stats.at<int>(i, cv::CC_STAT_LEFT);
    const int y = stats.at<int>(i, cv::CC_STAT_TOP);
    const int w = stats.at<int>(i, cv::CC_STAT_WIDTH);
    const int h = stats.at<int>(i, cv::CC_STAT_HEIGHT);
    const int area = stats.at<int>(i, cv::CC_STAT_AREA);
    if (area < kRedMinPix) {
      continue;
    }
    const double cx = centroids.at<double>(i, 0) + x0;
    const double cy = centroids.at<double>(i, 1) + y0;
    if (cy - lcy < kBarBelowMinRel * lscale) {
      continue;
    }
    if (std::abs(cx - lcx) > kBarXMaxRel * lscale) {
      continue;
    }
    if (std::max(w, h) > 1.5 * kBarLenMaxRel * lscale) {
      continue;
    }
    const auto contour = componentContour(labels, i, cv::Rect(x, y, w, h));
    if (!contour || contour->size() < 5) {
      continue;
    }
    const cv::RotatedRect rr = cv::minAreaRect(*contour);
    const double rw = rr.size.width;
    const double rh = rr.size.height;
    const double long_side = std::max(rw, rh);
    const double short_side = std::min(rw, rh);
    if (short_side < 1.0 || long_side / short_side < kBarAspectMin) {
      continue;
    }
    if (long_side < kBarLenMinRel * lscale ||
        long_side > kBarLenMaxRel * lscale) {
      continue;
    }
    const double tilt = tiltFromVertical(*contour);
    if (tilt > kBarTiltMaxDeg) {
      continue;
    }
    Bar bar;
    bar.x = x + x0;
    bar.y = y + y0;
    bar.w = w;
    bar.h = h;
    bar.cx = cx;
    bar.cy = cy;
    bar.tilt = tilt;
    out.push_back(bar);
  }
  return out;
}

std::optional<Pair> RGDetector::choosePair(const std::vector<Bar>& bars,
                                           const Lamp& lamp) const {
  std::optional<Pair> best;
  double best_score = std::numeric_limits<double>::max();
  const double lcx = lamp.cx;
  const double lcy = lamp.cy;
  const double lscale = lamp.scale;
  for (std::size_t i = 0; i < bars.size(); ++i) {
    for (std::size_t j = i + 1; j < bars.size(); ++j) {
      const Bar& a = bars[i];
      const Bar& b = bars[j];
      if ((a.cx - lcx) * (b.cx - lcx) >= 0.0) {
        continue;
      }
      const double sep = std::abs(a.cx - b.cx);
      const double sep_rel = sep / lscale;
      if (sep_rel < kPairSepMinRel || sep_rel > kPairSepMaxRel) {
        continue;
      }
      const double hsim = std::min(a.h, b.h) /
                          static_cast<double>(std::max(a.h, b.h));
      if (hsim < kPairHsimMin) {
        continue;
      }
      const double hmean = 0.5 * (a.h + b.h);
      if (std::abs(a.cy - b.cy) > kPairYdifMaxRel * hmean) {
        continue;
      }
      if (std::abs(a.tilt - b.tilt) > kPairTiltDifMax) {
        continue;
      }
      const double midx = 0.5 * (a.cx + b.cx);
      const double midy = 0.5 * (a.cy + b.cy);
      if (std::abs(midx - lcx) > kPairMidxMaxRel * lscale) {
        continue;
      }
      if (midy - lcy < kPairMidyMinRel * lscale) {
        continue;
      }
      const double score = (midy - lcy) / lscale +
                           1.5 * std::abs(midx - lcx) / lscale +
                           std::abs(sep_rel - kPairSepPrefRel) -
                           0.5 * hsim;
      if (score < best_score) {
        best_score = score;
        best = Pair{a, b, sep};
      }
    }
  }
  return best;
}

RoiDetection RGDetector::buildPairRoi(const Lamp& lamp, const Pair& pair) const {
  RoiDetection roi;
  roi.valid = true;
  roi.cx = 0.5 * (pair.a.cx + pair.b.cx);
  roi.cy = 0.5 * (pair.a.cy + pair.b.cy) +
           kRoiCyShiftOverSep * pair.sep;
  roi.w = kPlateWOverSep * pair.sep;
  roi.h = kPlateHOverBarH * std::max(pair.a.h, pair.b.h);
  const double hsim = std::min(pair.a.h, pair.b.h) /
                      static_cast<double>(std::max(pair.a.h, pair.b.h));
  const double par = 1.0 - std::min(
      1.0, std::abs(pair.a.tilt - pair.b.tilt) / kPairTiltDifMax);
  const double sym = 1.0 - std::min(
      1.0, std::abs(roi.cx - lamp.cx) / (kPairMidxMaxRel * lamp.scale));
  roi.method = "PAIR";
  roi.confidence = 0.5 + 0.2 * hsim + 0.15 * par + 0.15 * sym;
  return roi;
}

RoiDetection RGDetector::lampRoi(const Lamp& lamp) const {
  RoiDetection roi;
  roi.valid = true;
  roi.cx = lamp.cx;
  roi.cy = lamp.cy + kLampRoiDyRel * lamp.scale;
  roi.w = kLampRoiWRel * lamp.scale;
  roi.h = kLampRoiHRel * lamp.scale;
  roi.method = "RG_LAMP";
  roi.confidence = 0.2;
  return roi;
}

void RGDetector::updateTemplate(const Lamp& lamp, const Pair& pair,
                                const RoiDetection& roi) {
  const double s = lamp.scale;
  if (s < 1.0) {
    return;
  }
  const Bar* rl = pair.a.cx <= pair.b.cx ? &pair.a : &pair.b;
  const Bar* rr = pair.a.cx <= pair.b.cx ? &pair.b : &pair.a;
  Sample smp;
  smp.uL = cv::Vec2d((rl->cx - lamp.cx) / s, (rl->cy - lamp.cy) / s);
  smp.uR = cv::Vec2d((rr->cx - lamp.cx) / s, (rr->cy - lamp.cy) / s);
  smp.d = cv::Vec2d((roi.cx - lamp.cx) / s, (roi.cy - lamp.cy) / s);
  smp.ws = roi.w / s;
  smp.hs = roi.h / s;
  smp.barh = 0.5 * (rl->h + rr->h) / s;
  hist_.push_back(smp);
  while (static_cast<int>(hist_.size()) > params_.max_pairs) {
    hist_.pop_front();
  }
  last_pair_idx_ = frame_idx_;
}

std::optional<RGDetector::Stats> RGDetector::stats() const {
  if (static_cast<int>(hist_.size()) < params_.min_pairs) {
    return std::nullopt;
  }
  Stats st;
  cv::Vec2d uL_sum(0, 0), uR_sum(0, 0), d_sum(0, 0);
  double ws_sum = 0, hs_sum = 0, barh_sum = 0;
  for (const auto& s : hist_) {
    uL_sum += s.uL;
    uR_sum += s.uR;
    d_sum += s.d;
    ws_sum += s.ws;
    hs_sum += s.hs;
    barh_sum += s.barh;
  }
  const double n = static_cast<double>(hist_.size());
  st.uL_mean = uL_sum * (1.0 / n);
  st.uR_mean = uR_sum * (1.0 / n);
  st.d_mean = d_sum * (1.0 / n);
  st.ws = ws_sum / n;
  st.hs = hs_sum / n;
  st.barh = barh_sum / n;
  cv::Matx22d covL(0, 0, 0, 0), covR(0, 0, 0, 0);
  for (const auto& s : hist_) {
    const cv::Vec2d dl = s.uL - st.uL_mean;
    const cv::Vec2d dr = s.uR - st.uR_mean;
    covL(0, 0) += dl[0] * dl[0];
    covL(0, 1) += dl[0] * dl[1];
    covL(1, 0) += dl[1] * dl[0];
    covL(1, 1) += dl[1] * dl[1];
    covR(0, 0) += dr[0] * dr[0];
    covR(0, 1) += dr[0] * dr[1];
    covR(1, 0) += dr[1] * dr[0];
    covR(1, 1) += dr[1] * dr[1];
  }
  const double denom = std::max(1.0, n - 1.0);
  covL = covL * (1.0 / denom);
  covR = covR * (1.0 / denom);
  covL(0, 0) += params_.cov_reg;
  covL(1, 1) += params_.cov_reg;
  covR(0, 0) += params_.cov_reg;
  covR(1, 1) += params_.cov_reg;
  st.uL_cov = covL;
  st.uR_cov = covR;
  return st;
}

bool RGDetector::templateValid() const {
  return static_cast<int>(hist_.size()) >= params_.min_pairs &&
         frame_idx_ - last_pair_idx_ <= params_.max_template_age;
}

RoiDetection RGDetector::fallback(const Lamp& lamp,
                                  const std::vector<Bar>& bars) {
  if (!templateValid()) {
    return lampRoi(lamp);
  }
  const auto st_opt = stats();
  if (!st_opt) {
    return lampRoi(lamp);
  }
  const Stats& st = *st_opt;
  const double s = lamp.scale;
  const double gx = lamp.cx;
  const double gy = lamp.cy;
  const cv::Vec2d d = st.d_mean;
  const cv::Vec2d cg(gx + s * d[0], gy + s * d[1]);

  bool found = false;
  double best_m2 = std::numeric_limits<double>::max();
  const Bar* best_bar = nullptr;
  const cv::Vec2d* best_umean = nullptr;
  for (const auto& bar : bars) {
    const bool left = bar.cx < gx;
    const cv::Vec2d& u_mean = left ? st.uL_mean : st.uR_mean;
    const cv::Matx22d& cov = left ? st.uL_cov : st.uR_cov;
    const cv::Vec2d u((bar.cx - gx) / s, (bar.cy - gy) / s);
    const cv::Vec2d diff = u - u_mean;
    const cv::Matx22d inv = inverse2(cov);
    const double m2 = diff[0] * (inv(0, 0) * diff[0] + inv(0, 1) * diff[1]) +
                      diff[1] * (inv(1, 0) * diff[0] + inv(1, 1) * diff[1]);
    if (!std::isfinite(m2)) {
      continue;
    }
    const double barh = bar.h / s;
    const double h_rel = std::abs(barh - st.barh) / std::max(1e-6, st.barh);
    if (h_rel > 0.6) {
      continue;
    }
    const double dy = u[1] - u_mean[1];
    if (std::abs(dy) > params_.dy_max) {
      continue;
    }
    if (!found || m2 < best_m2) {
      found = true;
      best_m2 = m2;
      best_bar = &bar;
      best_umean = &u_mean;
    }
  }
  if (!found || std::sqrt(std::max(0.0, best_m2)) > params_.mahal_max) {
    return lampRoi(lamp);
  }
  const double e = std::sqrt(std::max(0.0, best_m2));
  const cv::Vec2d cr(best_bar->cx + s * (d[0] - (*best_umean)[0]),
                     best_bar->cy + s * (d[1] - (*best_umean)[1]));
  const double rel_r = std::exp(-0.5 * e * e);
  const double lambda = 1.0 / (1.0 + rel_r);
  RoiDetection roi;
  roi.valid = true;
  roi.cx = lambda * cg[0] + (1.0 - lambda) * cr[0];
  roi.cy = lambda * cg[1] + (1.0 - lambda) * cr[1];
  roi.w = s * st.ws;
  roi.h = s * st.hs;
  roi.method = "RG_SINGLE";
  roi.confidence = rel_r;
  return roi;
}

RoiDetection RGDetector::detect(const cv::Mat& bgr) {
  ++frame_idx_;
  if (bgr.empty()) {
    return RoiDetection{};
  }
  cv::Mat hsv;
  cv::cvtColor(bgr, hsv, cv::COLOR_BGR2HSV);
  const Lamp lamp = greenLamp(hsv);
  if (lamp.scale <= 0.0) {
    return RoiDetection{};
  }
  const std::vector<Bar> bars = redBars(hsv, lamp);
  const auto pair = choosePair(bars, lamp);
  if (pair) {
    RoiDetection roi = buildPairRoi(lamp, *pair);
    updateTemplate(lamp, *pair, roi);
    return roi;
  }
  return fallback(lamp, bars);
}

}  // namespace rm2027_roi
