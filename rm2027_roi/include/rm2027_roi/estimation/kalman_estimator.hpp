#pragma once

#include <Eigen/Dense>

#include "rm2027_roi/core/types.hpp"
#include "rm2027_roi/core/parameters.hpp"

namespace rm2027_roi {

class KalmanEstimator {
 public:
  explicit KalmanEstimator(const KalmanParams& params = KalmanParams());

  void reset();
  RoiEstimate update(const RoiDetection& detection, double ts);

  const KalmanParams& params() const { return params_; }
  const Eigen::Matrix<double, 8, 8>& covariance() const { return P_; }

 private:
  void initFrom(const RoiDetection& detection, double ts);
  void predict(double dt);
  RoiEstimate output(const std::string& state, const std::string& source,
                     double horizon, double cost_ms) const;
  static double methodRScale(const RoiDetection& detection);

  KalmanParams params_;
  Eigen::Matrix<double, 8, 1> x_;
  Eigen::Matrix<double, 8, 8> P_;
  Eigen::Matrix<double, 4, 4> R_;
  bool initialized_ = false;
  bool lost_ = false;
  int miss_count_ = 0;
  int rejects_ = 0;
  double last_measure_ts_ = 0.0;
  double last_state_ts_ = 0.0;
};

}  // namespace rm2027_roi
