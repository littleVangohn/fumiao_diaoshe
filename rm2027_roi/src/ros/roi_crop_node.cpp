#include "rm2027_roi/ros/roi_crop_node.hpp"

#include <opencv2/imgproc.hpp>

namespace rm2027_roi {

RoiCropNode::RoiCropNode() : Node("roi_crop") {
  bg_qp_ = declare_parameter<int>("bg_qp", bg_qp_);
  fg_qp_ = declare_parameter<int>("fg_qp", fg_qp_);
  canvas_pub_ = create_publisher<sensor_msgs::msg::Image>("roi/canvas", 10);
  qmap_pub_ = create_publisher<std_msgs::msg::Int8MultiArray>("roi/qmap", 10);
  roi_pub_ = create_publisher<std_msgs::msg::UInt8MultiArray>(
      "roi/normalized", 10);
  image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      "image_raw", 10,
      std::bind(&RoiCropNode::onImage, this, std::placeholders::_1));
  est_sub_ = create_subscription<rm2027_roi::msg::RoiEstimate>(
      "roi/estimate", 10,
      std::bind(&RoiCropNode::onEstimate, this, std::placeholders::_1));
}

void RoiCropNode::onImage(
    const sensor_msgs::msg::Image::SharedPtr msg) {
  try {
    std::lock_guard<std::mutex> lock(mutex_);
    latest_ = cv_bridge::toCvCopy(msg, "bgr8");
  } catch (const std::exception& e) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                         "image conversion failed: %s", e.what());
  }
}

void RoiCropNode::onEstimate(
    const rm2027_roi::msg::RoiEstimate::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!latest_) {
    return;
  }
  RoiDetection roi;
  if (msg->valid && msg->w > 0.0 && msg->h > 0.0) {
    roi.valid = true;
    roi.cx = msg->cx;
    roi.cy = msg->cy;
    roi.w = msg->w;
    roi.h = msg->h;
    roi.method = msg->source;
    last_roi_ = roi;
  } else if (last_roi_.valid) {
    roi = last_roi_;
  } else {
    return;
  }

  const Crop crop = crop4x3(roi, latest_->image.cols, latest_->image.rows);
  const cv::Rect rect(crop.x0, crop.y0, crop.w, crop.h);
  cv::Mat canvas;
  const int interpolation = (kCanvasW <= crop.w) ? cv::INTER_AREA
                                                 : cv::INTER_LINEAR;
  cv::resize(latest_->image(rect), canvas, cv::Size(kCanvasW, kCanvasH), 0,
             0, interpolation);

  const auto roi_canvas = roiOnCanvas(roi, crop);
  const Qpmap qmap = buildQpmap(roi_canvas, kCanvasW, kCanvasH, bg_qp_,
                               fg_qp_);
  const auto norm = normalizedRoi(roi_canvas);

  cv_bridge::CvImage canvas_msg(msg->header, "bgr8", canvas);
  canvas_pub_->publish(*canvas_msg.toImageMsg());

  std_msgs::msg::Int8MultiArray qmsg;
  qmsg.layout.dim.resize(2);
  qmsg.layout.dim[0].label = "rows";
  qmsg.layout.dim[0].size = Qpmap::H;
  qmsg.layout.dim[0].stride = Qpmap::W;
  qmsg.layout.dim[1].label = "cols";
  qmsg.layout.dim[1].size = Qpmap::W;
  qmsg.layout.dim[1].stride = 1;
  qmsg.data.assign(qmap.data.begin(), qmap.data.end());
  qmap_pub_->publish(qmsg);

  std_msgs::msg::UInt8MultiArray rmsg;
  rmsg.data.assign(norm.begin(), norm.end());
  roi_pub_->publish(rmsg);
}

}  // namespace rm2027_roi

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<rm2027_roi::RoiCropNode>());
  rclcpp::shutdown();
  return 0;
}
