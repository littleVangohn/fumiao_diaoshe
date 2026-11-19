#pragma once

#include <rclcpp/rclcpp.hpp>

#include "rm2027_roi/estimation/kalman_estimator.hpp"
#include "rm2027_roi/msg/roi_detection.hpp"
#include "rm2027_roi/msg/roi_estimate.hpp"

namespace rm2027_roi {

class RoiEstimatorNode : public rclcpp::Node {
 public:
  RoiEstimatorNode();

 private:
  void onDetection(
      const rm2027_roi::msg::RoiDetection::SharedPtr msg);

  KalmanEstimator estimator_;
  rclcpp::Subscription<rm2027_roi::msg::RoiDetection>::SharedPtr sub_;
  rclcpp::Publisher<rm2027_roi::msg::RoiEstimate>::SharedPtr pub_;
};

}  // namespace rm2027_roi
