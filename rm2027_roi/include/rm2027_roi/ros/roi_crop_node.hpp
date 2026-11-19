#pragma once

#include <mutex>

#include <cv_bridge/cv_bridge.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/int8_multi_array.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>

#include "rm2027_roi/coding/crop_qpmap.hpp"
#include "rm2027_roi/msg/roi_estimate.hpp"

namespace rm2027_roi {

class RoiCropNode : public rclcpp::Node {
 public:
  RoiCropNode();

 private:
  void onImage(const sensor_msgs::msg::Image::SharedPtr msg);
  void onEstimate(
      const rm2027_roi::msg::RoiEstimate::SharedPtr msg);

  std::mutex mutex_;
  cv_bridge::CvImagePtr latest_;
  RoiDetection last_roi_;
  int bg_qp_ = kBgQp;
  int fg_qp_ = kFgQp;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<rm2027_roi::msg::RoiEstimate>::SharedPtr est_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr canvas_pub_;
  rclcpp::Publisher<std_msgs::msg::Int8MultiArray>::SharedPtr qmap_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr roi_pub_;
};

}  // namespace rm2027_roi
