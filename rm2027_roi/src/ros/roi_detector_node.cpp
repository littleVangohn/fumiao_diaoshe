#include "rm2027_roi/ros/roi_detector_node.hpp"

#include <cv_bridge/cv_bridge.h>

namespace rm2027_roi {

RoiDetectorNode::RoiDetectorNode() : Node("roi_detector") {
  RgParams params;
  params.max_pairs = declare_parameter<int>("max_pairs", params.max_pairs);
  params.min_pairs = declare_parameter<int>("min_pairs", params.min_pairs);
  params.max_template_age =
      declare_parameter<int>("max_template_age", params.max_template_age);
  params.cov_reg = declare_parameter<double>("cov_reg", params.cov_reg);
  params.mahal_max = declare_parameter<double>("mahal_max", params.mahal_max);
  params.dy_max = declare_parameter<double>("dy_max", params.dy_max);
  detector_ = std::make_unique<RGDetector>(params);

  pub_ = create_publisher<rm2027_roi::msg::RoiDetection>(
      "roi/detection", 10);
  sub_ = create_subscription<sensor_msgs::msg::Image>(
      "image_raw", 10,
      std::bind(&RoiDetectorNode::onImage, this, std::placeholders::_1));
}

void RoiDetectorNode::onImage(
    const sensor_msgs::msg::Image::SharedPtr msg) {
  try {
    const auto view = cv_bridge::toCvCopy(msg, "bgr8");
    const RoiDetection det = detector_->detect(view->image);
    rm2027_roi::msg::RoiDetection out;
    out.header = msg->header;
    out.valid = det.valid;
    out.cx = det.cx;
    out.cy = det.cy;
    out.w = det.w;
    out.h = det.h;
    out.method = det.method;
    out.confidence = det.confidence;
    pub_->publish(out);
  } catch (const std::exception& e) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                         "image conversion/detection failed: %s", e.what());
  }
}

}  // namespace rm2027_roi

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<rm2027_roi::RoiDetectorNode>());
  rclcpp::shutdown();
  return 0;
}
