#include "rm2027_roi/estimation/kalman_estimator.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>

namespace rm2027_roi {
namespace {

using Vec8 = Eigen::Matrix<double, 8, 1>;
using Mat8 = Eigen::Matrix<double, 8, 8>;
using Vec4 = Eigen::Matrix<double, 4, 1>;
using Mat4 = Eigen::Matrix<double, 4, 4>;
using Mat48 = Eigen::Matrix<double, 4, 8>;
using Mat84 = Eigen::Matrix<double, 8, 4>;

Mat8 F(double dt) {
  Mat8 f = Mat8::Identity();
  f(0, 2) = dt;
  f(1, 3) = dt;
  f(4, 6) = dt;
  f(5, 7) = dt;
  return f;
}

Mat8 Q(double dt, double q_pos, double q_size) {
  dt = std::max(dt, 0.0);
  const double a = dt * dt * dt / 3.0;
  const double b = dt * dt / 2.0;
  const double c = dt;
  Mat8 q = Mat8::Zero();
  const int pairs[4][4] = {{0, 2, 0, 0}, {1, 3, 0, 0},
                           {4, 6, 0, 0}, {5, 7, 0, 0}};
  for (int k = 0; k < 4; ++k) {
    const int p = pairs[k][0];
    const int v = pairs[k][1];
    const double qv = (k < 2) ? q_pos : q_size;
    q(p, p) = qv * a;
    q(p, v) = qv * b;
    q(v, p) = qv * b;
    q(v, v) = qv * c;
  }
  return q;
}

Mat48 H() {
  Mat48 h = Mat48::Zero();
  h(0, 0) = 1.0;
  h(1, 1) = 1.0;
  h(2, 4) = 1.0;
  h(3, 5) = 1.0;
  return h;
}

double logSize(double v) { return std::log(std::max(v, 1e-6)); }

double expSize(double v) {
  return std::exp(std::min(std::max(v, -20.0), 20.0));
}

}  // namespace

KalmanEstimator::KalmanEstimator(const KalmanParams& params)
    : params_(params) {
  R_ = Mat4::Zero();
  R_(0, 0) = params_.r_pos;
  R_(1, 1) = params_.r_pos;
  R_(2, 2) = params_.r_size;
  R_(3, 3) = params_.r_size;
  reset();
}

void KalmanEstimator::reset() {
  x_ = Vec8::Zero();
  P_ = Mat8::Zero();
  initialized_ = false;
  lost_ = false;
  miss_count_ = 0;
  rejects_ = 0;
  last_measure_ts_ = 0.0;
  last_state_ts_ = 0.0;
}

void KalmanEstimator::initFrom(const RoiDetection& detection, double ts) {
  x_ << detection.cx, detection.cy, 0.0, 0.0,
      logSize(detection.w), logSize(detection.h), 0.0, 0.0;
  P_ = Mat8::Zero();
  P_(0, 0) = P_(1, 1) = params_.r_pos;
  P_(2, 2) = P_(3, 3) = params_.p0_vel * params_.p0_vel;
  P_(4, 4) = P_(5, 5) = params_.r_size;
  P_(6, 6) = P_(7, 7) = params_.p0_size_vel * params_.p0_size_vel;
  initialized_ = true;
  lost_ = false;
  miss_count_ = 0;
  rejects_ = 0;
  last_measure_ts_ = ts;
  last_state_ts_ = ts;
}

void KalmanEstimator::predict(double dt) {
  const Mat8 f = F(dt);
  x_ = f * x_;
  P_ = f * P_ * f.transpose() + Q(dt, params_.q_pos, params_.q_size);
  P_ = 0.5 * (P_ + P_.transpose());
}

double KalmanEstimator::methodRScale(const RoiDetection& detection) {
  if (detection.method.empty() || detection.method == "PAIR") {
    return 1.0;
  }
  if (detection.method == "RG_SINGLE") {
    const double conf = std::min(std::max(detection.confidence, 0.5), 1.0);
    return 2.0 / conf;
  }
  if (detection.method == "RG_LAMP") {
    return 3.0;
  }
  return 12.0;
}

RoiEstimate KalmanEstimator::output(const std::string& state,
                                    const std::string& source,
                                    double horizon, double cost_ms) const {
  RoiEstimate est;
  const double w = expSize(x_(4));
  const double h = expSize(x_(5));
  const double unc = std::sqrt(std::max(P_(0, 0) + P_(1, 1), 0.0));
  const bool finite = std::isfinite(x_(0)) && std::isfinite(x_(1)) &&
                      std::isfinite(w) && std::isfinite(h) &&
                      std::isfinite(unc);
  est.cost_ms = cost_ms;
  if (!finite || w <= 0.0 || h <= 0.0) {
    est.valid = false;
    est.state = "LOST";
    est.source = "NONE";
    est.horizon = 0.0;
    return est;
  }
  est.valid = true;
  est.cx = x_(0);
  est.cy = x_(1);
  est.w = w;
  est.h = h;
  est.state = state;
  est.source = source;
  est.horizon = horizon;
  est.uncertainty = unc;
  return est;
}

RoiEstimate KalmanEstimator::update(const RoiDetection& detection,
                                    double ts) {
  const auto t0 = std::chrono::steady_clock::now();
  const auto elapsed_ms = [&t0]() {
    return std::chrono::duration<double, std::milli>(
               std::chrono::steady_clock::now() - t0)
        .count();
  };

  if (detection.valid) {
    if (!initialized_ || lost_) {
      initFrom(detection, ts);
      RoiEstimate e;
      e.valid = true;
      e.cx = x_(0);
      e.cy = x_(1);
      e.w = expSize(x_(4));
      e.h = expSize(x_(5));
      e.state = "TRACKING";
      e.source = "MEASUREMENT";
      e.uncertainty = std::sqrt(std::max(P_(0, 0) + P_(1, 1), 0.0));
      e.cost_ms = elapsed_ms();
      return e;
    }

    const double dt = std::max(ts - last_state_ts_, 0.0);
    predict(dt);
    last_state_ts_ = ts;
    last_measure_ts_ = ts;

    Vec4 z;
    z << detection.cx, detection.cy, logSize(detection.w),
        logSize(detection.h);
    const Mat48 hmat = H();
    const Vec4 y = z - hmat * x_;
    const Mat4 r_eff = R_ * methodRScale(detection);
    const Mat4 s = hmat * P_ * hmat.transpose() + r_eff;
    Eigen::FullPivLU<Mat4> lu(s);
    const bool invertible = lu.isInvertible();
    Mat4 s_inv = Mat4::Identity();
    if (invertible) {
      s_inv = lu.inverse();
    }
    double d2 = std::numeric_limits<double>::infinity();
    if (invertible) {
      d2 = y.dot(s_inv * y);
    }

    if (!std::isfinite(d2) || d2 > params_.gate_chi2) {
      ++rejects_;
      const int reinit_n = (detection.method.empty() ||
                            detection.method == "PAIR")
                               ? params_.gate_reinit
                               : std::max(params_.gate_reinit, 8);
      if (rejects_ >= reinit_n) {
        initFrom(detection, ts);
        RoiEstimate e;
        e.valid = true;
        e.cx = x_(0);
        e.cy = x_(1);
        e.w = expSize(x_(4));
        e.h = expSize(x_(5));
        e.state = "TRACKING";
        e.source = "MEASUREMENT";
        e.uncertainty = std::sqrt(std::max(P_(0, 0) + P_(1, 1), 0.0));
        e.cost_ms = elapsed_ms();
        return e;
      }
      return output("DEGRADED", "PREDICTION", 0.0, elapsed_ms());
    }

    rejects_ = 0;
    const Mat84 k = P_ * hmat.transpose() * s_inv;
    x_ = x_ + k * y;
    const Mat8 i_kh = Mat8::Identity() - k * hmat;
    P_ = i_kh * P_ * i_kh.transpose() + k * r_eff * k.transpose();
    P_ = 0.5 * (P_ + P_.transpose());
    RoiEstimate e;
    e.valid = true;
    e.cx = x_(0);
    e.cy = x_(1);
    e.w = expSize(x_(4));
    e.h = expSize(x_(5));
    e.state = "TRACKING";
    e.source = "MEASUREMENT";
    e.uncertainty = std::sqrt(std::max(P_(0, 0) + P_(1, 1), 0.0));
    e.cost_ms = elapsed_ms();
    if (!std::isfinite(e.cx) || !std::isfinite(e.cy) ||
        !std::isfinite(e.w) || !std::isfinite(e.h)) {
      return output("LOST", "NONE", 0.0, elapsed_ms());
    }
    return e;
  }

  if (!initialized_) {
    RoiEstimate e;
    e.valid = false;
    e.state = "LOST";
    e.source = "NONE";
    e.cost_ms = elapsed_ms();
    return e;
  }
  ++miss_count_;
  if (miss_count_ <= params_.hold) {
    const double dt = std::max(ts - last_state_ts_, 0.0);
    predict(dt);
    last_state_ts_ = ts;
    const double horizon = ts - last_measure_ts_;
    RoiEstimate e;
    e.valid = true;
    e.cx = x_(0);
    e.cy = x_(1);
    e.w = expSize(x_(4));
    e.h = expSize(x_(5));
    e.state = "DEGRADED";
    e.source = "PREDICTION";
    e.horizon = horizon;
    e.uncertainty = std::sqrt(std::max(P_(0, 0) + P_(1, 1), 0.0));
    e.cost_ms = elapsed_ms();
    if (!std::isfinite(e.cx) || !std::isfinite(e.cy) ||
        !std::isfinite(e.w) || !std::isfinite(e.h)) {
      return output("LOST", "NONE", 0.0, elapsed_ms());
    }
    return e;
  }
  lost_ = true;
  RoiEstimate e;
  e.valid = false;
  e.state = "LOST";
  e.source = "NONE";
  e.cost_ms = elapsed_ms();
  return e;
}

}  // namespace rm2027_roi
