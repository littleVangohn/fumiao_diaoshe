#include <gtest/gtest.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include "rm2027_roi/core/roi_pipeline.hpp"

using namespace rm2027_roi;

TEST(Pipeline, FallbackOnEmptyFrame) {
  RoiPipeline pipe;
  cv::Mat black = cv::Mat::zeros(1080, 1440, CV_8UC3);
  FrameResult result;
  const auto est = pipe.process(black, 0.0, &result);
  EXPECT_FALSE(est.valid);
  EXPECT_TRUE(result.valid_crop);
  EXPECT_EQ(result.crop.w, 400);
  EXPECT_EQ(result.crop.h, 300);
  int fg = 0;
  for (auto v : result.qmap.data) {
    if (v == kFgQp) {
      ++fg;
    }
  }
  EXPECT_GT(fg, 0);
}

TEST(Pipeline, SyntheticPairTracks) {
  cv::Mat img = cv::Mat::zeros(1080, 1440, CV_8UC3);
  cv::ellipse(img, cv::Point(720, 500), cv::Size(22, 16), 0, 0, 360,
              cv::Scalar(0, 255, 0), cv::FILLED);
  cv::ellipse(img, cv::Point(703, 554), cv::Size(5, 14), 0, 0, 360,
              cv::Scalar(0, 0, 255), cv::FILLED);
  cv::ellipse(img, cv::Point(737, 554), cv::Size(5, 14), 0, 0, 360,
              cv::Scalar(0, 0, 255), cv::FILLED);
  RoiPipeline pipe;
  FrameResult result;
  const auto est = pipe.process(img, 0.0, &result);
  EXPECT_TRUE(est.valid);
  EXPECT_TRUE(result.detection.valid);
  EXPECT_EQ(result.detection.method, "PAIR");
  EXPECT_TRUE(result.valid_crop);
  EXPECT_NEAR(est.cx, 720.0, 3.0);
}
