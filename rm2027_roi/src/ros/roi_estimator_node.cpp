#include "rm2027_roi/ros/roi_estimator_node.hpp"

namespace rm2027_roi {

RoiEstimatorNode::RoiEstimatorNode()
    : Node("roi_estimator"), estimator_(KalmanParams{}) {
  KalmanParams params;
  params.q_pos = declare_parameter<double>("q_pos", params.q_pos);
  params.q_size = declare_parameter<double>("q_size", params.q_size);
  params.r_pos = declare_parameter<double>("r_pos", params.r_pos);
  params.r_size = declare_parameter<double>("r_size", params.r_size);
  params.gate_chi2 =
      declare_parameter<double>("gate_chi2", params.gate_chi2);
  params.gate_reinit =
      declare_parameter<int>("gate_reinit", params.gate_reinit);
  params.p0_vel = declare_parameter<double>("p0_vel", params.p0_vel);
  params.p0_size_vel =
      declare_parameter<double>("p0_size_vel", params.p0_size_vel);
  params.hold = declare_parameter<int>("hold", params.hold);
  estimator_ = KalmanEstimator(params);

  pub_ = create_publisher<rm2027_roi::msg::RoiEstimate>(
      "roi/estimate", 10);
  sub_ = create_subscription<rm2027_roi::msg::RoiDetection>(
      "roi/detection", 10,
      std::bind(&RoiEstimatorNode::onDetection, this,
                std::placeholders::_1));
}

void RoiEstimatorNode::onDetection(
    const rm2027_roi::msg::RoiDetection::SharedPtr msg) {
  RoiDetection det;
  det.valid = msg->valid;
  det.cx = msg->cx;
  det.cy = msg->cy;
  det.w = msg->w;
  det.h = msg->h;
  det.method = msg->method;
  det.confidence = msg->confidence;
  double ts = rclcpp::Time(msg->header.stamp).seconds();
  if (ts <= 0.0) {
    ts = now().seconds();
  }
  const RoiEstimate est = estimator_.update(det, ts);
  rm2027_roi::msg::RoiEstimate out;
  out.header = msg->header;
  out.valid = est.valid;
  out.cx = est.cx;
  out.cy = est.cy;
  out.w = est.w;
  out.h = est.h;
  out.track_state = est.state;
  out.source = est.source;
  out.prediction_horizon_s = est.horizon;
  out.uncertainty = est.uncertainty;
  out.update_cost_ms = est.cost_ms;
  pub_->publish(out);
}

}  // namespace rm2027_roi

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<rm2027_roi::RoiEstimatorNode>());
  rclcpp::shutdown();
  return 0;
}
