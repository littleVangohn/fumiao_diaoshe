#pragma once

#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "rm2027_roi/detection/rg_detector.hpp"
#include "rm2027_roi/msg/roi_detection.hpp"

namespace rm2027_roi {

class RoiDetectorNode : public rclcpp::Node {
 public:
  RoiDetectorNode();

 private:
  void onImage(const sensor_msgs::msg::Image::SharedPtr msg);

  std::unique_ptr<RGDetector> detector_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
  rclcpp::Publisher<rm2027_roi::msg::RoiDetection>::SharedPtr pub_;
};

}  // namespace rm2027_roi
