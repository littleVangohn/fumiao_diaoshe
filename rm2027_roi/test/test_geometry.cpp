#include <cmath>

#include <gtest/gtest.h>
#include <opencv2/imgproc.hpp>

#include "rm2027_roi/detection/rg_detector.hpp"

using namespace rm2027_roi;

TEST(Geometry, PairRoi) {
  Lamp lamp;
  lamp.cx = 720.0;
  lamp.cy = 500.0;
  lamp.scale = 40.0;
  Pair pair;
  pair.a.cx = 700.0;
  pair.a.cy = 560.0;
  pair.a.h = 30;
  pair.a.tilt = 0.0;
  pair.b.cx = 740.0;
  pair.b.cy = 560.0;
  pair.b.h = 30;
  pair.b.tilt = 0.0;
  pair.sep = 40.0;
  RGDetector det;
  const auto roi = det.buildPairRoi(lamp, pair);
  EXPECT_TRUE(roi.valid);
  EXPECT_NEAR(roi.cx, 720.0, 1e-9);
  EXPECT_NEAR(roi.cy, 562.0, 1e-9);
  EXPECT_NEAR(roi.w, 43.6, 1e-9);
  EXPECT_NEAR(roi.h, 65.4, 1e-9);
}

TEST(Geometry, LampRoi) {
  Lamp lamp;
  lamp.cx = 720.0;
  lamp.cy = 500.0;
  lamp.scale = 40.0;
  RGDetector det;
  const auto roi = det.lampRoi(lamp);
  EXPECT_TRUE(roi.valid);
  EXPECT_NEAR(roi.cx, 720.0, 1e-9);
  EXPECT_NEAR(roi.cy, 562.4, 1e-9);
  EXPECT_NEAR(roi.w, 52.0, 1e-9);
  EXPECT_NEAR(roi.h, 27.2, 1e-9);
}

TEST(Geometry, SyntheticDetectorPair) {
  cv::Mat img = cv::Mat::zeros(1080, 1440, CV_8UC3);
  cv::ellipse(img, cv::Point(720, 500), cv::Size(22, 16), 0, 0, 360,
              cv::Scalar(0, 255, 0), cv::FILLED);
  cv::ellipse(img, cv::Point(703, 554), cv::Size(5, 14), 0, 0, 360,
              cv::Scalar(0, 0, 255), cv::FILLED);
  cv::ellipse(img, cv::Point(737, 554), cv::Size(5, 14), 0, 0, 360,
              cv::Scalar(0, 0, 255), cv::FILLED);
  RGDetector det;
  const auto out = det.detect(img);
  EXPECT_TRUE(out.valid);
  EXPECT_EQ(out.method, "PAIR");
  EXPECT_NEAR(out.cx, 720.0, 2.0);
}
