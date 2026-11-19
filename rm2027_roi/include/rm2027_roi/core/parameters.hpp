#pragma once

namespace rm2027_roi {

struct KalmanParams {
  double q_pos = 1313.07;
  double q_size = 0.000510016;
  double r_pos = 25.3156;
  double r_size = 0.016075;
  double gate_chi2 = 16.0;
  int gate_reinit = 3;
  double p0_vel = 283.212;
  double p0_size_vel = 0.707658;
  int hold = 2;
};

struct RgParams {
  int max_pairs = 60;
  int min_pairs = 5;
  int max_template_age = 120;
  double cov_reg = 0.05 * 0.05;
  double mahal_max = 4.0;
  double dy_max = 0.8;
};

}  // namespace rm2027_roi
