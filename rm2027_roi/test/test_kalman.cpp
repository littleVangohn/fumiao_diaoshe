#include <algorithm>
#include <cmath>
#include <vector>

#include <gtest/gtest.h>

#include "rm2027_roi/estimation/kalman_estimator.hpp"

using namespace rm2027_roi;

namespace {

RoiDetection makeDet(double cx, double cy, double w, double h,
                     const std::string& method = "PAIR",
                     double conf = 1.0) {
  RoiDetection d;
  d.valid = true;
  d.cx = cx;
  d.cy = cy;
  d.w = w;
  d.h = h;
  d.method = method;
  d.confidence = conf;
  return d;
}

double median(std::vector<double> values) {
  std::sort(values.begin(), values.end());
  return values[values.size() / 2];
}

}  // namespace

TEST(Kalman, CVNoSteadyLag) {
  KalmanEstimator est(KalmanParams{});
  const double speed = 180.0;
  std::vector<double> lag_ms;
  for (int i = 0; i < 160; ++i) {
    const double ts = i / 30.0;
    const double cx = 300.0 + speed * ts;
    const auto e = est.update(makeDet(cx, 540.0, 64.0, 48.0), ts);
    ASSERT_TRUE(e.valid);
    if (i >= 100) {
      lag_ms.push_back((cx - e.cx) / speed * 1000.0);
    }
  }
  EXPECT_LT(std::abs(median(lag_ms)), 2.0);
}

TEST(Kalman, MissingBudgetAndLOST) {
  KalmanEstimator est(KalmanParams{});
  double ts = 0.0;
  for (int i = 0; i < 20; ++i) {
    est.update(makeDet(300.0 + 3.0 * i, 540.0, 64.0, 48.0), ts);
    ts += 1.0 / 30.0;
  }
  RoiDetection missing;
  auto e1 = est.update(missing, ts);
  EXPECT_TRUE(e1.valid);
  EXPECT_EQ(e1.state, "DEGRADED");
  EXPECT_EQ(e1.source, "PREDICTION");
  ts += 1.0 / 30.0;
  auto e2 = est.update(missing, ts);
  EXPECT_TRUE(e2.valid);
  ts += 1.0 / 30.0;
  auto e3 = est.update(missing, ts);
  EXPECT_FALSE(e3.valid);
  EXPECT_EQ(e3.state, "LOST");
}

TEST(Kalman, MethodAwareDownweightsLAMP) {
  KalmanEstimator pair_est(KalmanParams{});
  KalmanEstimator lamp_est(KalmanParams{});
  double ts = 0.0;
  for (int i = 0; i < 30; ++i) {
    const auto d = makeDet(300.0 + 3.0 * i, 540.0, 64.0, 48.0);
    pair_est.update(d, ts);
    lamp_est.update(d, ts);
    ts += 1.0 / 30.0;
  }
  const double prior = 300.0 + 3.0 * 29;
  const auto e_pair = pair_est.update(
      makeDet(prior + 10.0, 540.0, 64.0, 48.0, "PAIR", 1.0), ts);
  const auto e_lamp = lamp_est.update(
      makeDet(prior + 10.0, 540.0, 64.0, 48.0, "RG_LAMP", 0.2), ts);
  EXPECT_LT(std::abs(e_lamp.cx - prior), std::abs(e_pair.cx - prior));
}

TEST(Kalman, CovarianceStable) {
  KalmanEstimator est(KalmanParams{});
  double ts = 0.0;
  for (int i = 0; i < 300; ++i) {
    est.update(makeDet(300.0 + 3.0 * i, 540.0, 64.0, 48.0), ts);
    ts += 1.0 / 30.0;
  }
  const auto& p = est.covariance();
  EXPECT_TRUE(p.allFinite());
  EXPECT_LT((p - p.transpose()).cwiseAbs().maxCoeff(), 1e-9);
  EXPECT_GT(p(0, 0), 0.0);
}
