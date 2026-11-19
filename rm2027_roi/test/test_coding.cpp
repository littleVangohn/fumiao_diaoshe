#include <cstdint>
#include <vector>

#include <gtest/gtest.h>

#include "rm2027_roi/coding/crop_qpmap.hpp"
#include "rm2027_roi/coding/packetizer.hpp"

using namespace rm2027_roi;

TEST(Coding, Crop4x3) {
  RoiDetection roi;
  roi.cx = 720.0;
  roi.cy = 540.0;
  roi.w = 64.0;
  roi.h = 48.0;
  const Crop crop = crop4x3(roi, 1440, 1080);
  EXPECT_EQ(crop.w, 256);
  EXPECT_EQ(crop.h, 192);
  EXPECT_EQ(crop.x0, 592);
  EXPECT_EQ(crop.y0, 444);
}

TEST(Coding, CropClamp) {
  RoiDetection roi;
  roi.cx = 10.0;
  roi.cy = 10.0;
  roi.w = 4.0;
  roi.h = 3.0;
  const Crop crop = crop4x3(roi, 1440, 1080);
  EXPECT_EQ(crop.w, 192);
  EXPECT_EQ(crop.h, 144);
  EXPECT_EQ(crop.x0, 0);
  EXPECT_EQ(crop.y0, 0);
}

TEST(Coding, QpmapAndNormalizedRoi) {
  const std::array<double, 4> roi = {160.0, 120.0, 64.0, 48.0};
  const Qpmap q = buildQpmap(roi);
  EXPECT_EQ(q.data.size(), 300u);
  int fg = 0;
  for (auto v : q.data) {
    EXPECT_TRUE(v == kBgQp || v == kFgQp);
    if (v == kFgQp) {
      ++fg;
    }
  }
  EXPECT_GT(fg, 0);
  const auto norm = normalizedRoi({0.0, 0.0, 320.0, 240.0});
  EXPECT_EQ(norm[0], 0);
  EXPECT_EQ(norm[1], 0);
  EXPECT_EQ(norm[2], 255);
  EXPECT_EQ(norm[3], 255);
}

TEST(Coding, PacketizerRoundTrip) {
  std::vector<uint8_t> stream(1000);
  for (std::size_t i = 0; i < stream.size(); ++i) {
    stream[i] = static_cast<uint8_t>(i & 0xFF);
  }
  std::vector<Au> aus;
  const std::size_t sizes[3] = {400, 300, 300};
  std::size_t offset = 0;
  for (int i = 0; i < 3; ++i) {
    Au au;
    au.frame_id = static_cast<uint16_t>(i);
    au.start = offset;
    au.size = sizes[i];
    au.roi = {static_cast<uint8_t>(i + 1), 2, 3, 4};
    au.has_roi = true;
    aus.push_back(au);
    offset += sizes[i];
  }
  const auto packets = packetize(stream, aus);
  EXPECT_FALSE(packets.empty());
  std::vector<Packet> decoded;
  for (const auto& p : packets) {
    const auto raw = encodePacket(p);
    decoded.push_back(decodePacket(raw.data(), raw.size()));
  }
  const auto [rebuilt, rois] = reassemble(decoded);
  EXPECT_EQ(rebuilt, stream);
  EXPECT_EQ(rois.size(), 3u);
  EXPECT_EQ(rois.at(0)[0], 1);
  EXPECT_EQ(rois.at(2)[0], 3);
}
