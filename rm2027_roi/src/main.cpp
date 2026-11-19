#include <algorithm>
#include <array>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>

#include "rm2027_roi/coding/crop_qpmap.hpp"
#include "rm2027_roi/coding/packetizer.hpp"
#include "rm2027_roi/core/roi_pipeline.hpp"

namespace fs = std::filesystem;

namespace {

std::vector<std::string> split(const std::string& line, char delim) {
  std::vector<std::string> out;
  std::stringstream ss(line);
  std::string item;
  while (std::getline(ss, item, delim)) {
    out.push_back(item);
  }
  return out;
}

}  // namespace

int main(int argc, char** argv) {
  std::string source;
  std::string out_dir = "offline_out";
  std::string encoder;
  int max_frames = 0;
  double fps = 30.0;
  int bitrate_kbps = 88;
  int gop = 60;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    auto next = [&]() -> std::string {
      if (i + 1 >= argc) {
        throw std::runtime_error("missing value for " + arg);
      }
      return argv[++i];
    };
    if (arg == "--source") {
      source = next();
    } else if (arg == "--out") {
      out_dir = next();
    } else if (arg == "--frames") {
      max_frames = std::stoi(next());
    } else if (arg == "--fps") {
      fps = std::stod(next());
    } else if (arg == "--encoder") {
      encoder = next();
    } else if (arg == "--bitrate-kbps") {
      bitrate_kbps = std::stoi(next());
    } else if (arg == "--gop") {
      gop = std::stoi(next());
    } else if (arg == "--help") {
      std::cout << "usage: offline_demo --source video.mp4 --out dir "
                   "[--frames N] [--encoder qpmap_encoder]\n";
      return 0;
    }
  }
  if (source.empty()) {
    std::cerr << "--source is required\n";
    return 2;
  }
  fs::create_directories(out_dir);
  cv::VideoCapture cap(source);
  if (!cap.isOpened()) {
    std::cerr << "cannot open " << source << "\n";
    return 2;
  }
  const double source_fps = cap.get(cv::CAP_PROP_FPS);
  if (source_fps > 1.0) {
    fps = source_fps;
  }
  const int total = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_COUNT));
  int n = (max_frames > 0) ? std::min(max_frames, total) : total;
  if (n <= 0) {
    n = max_frames > 0 ? max_frames : 600;
  }

  const fs::path canvas_path = fs::path(out_dir) / "canvas_320x240.yuv";
  const fs::path qmap_path = fs::path(out_dir) / "qmap_320x240.bin";
  const fs::path est_path = fs::path(out_dir) / "estimates.csv";
  const fs::path stream_path = fs::path(out_dir) / "stream.h264";
  const fs::path enc_csv = fs::path(out_dir) / "encoder.csv";
  const fs::path packets_path = fs::path(out_dir) / "packets.bin";

  std::ofstream fyuv(canvas_path, std::ios::binary);
  std::ofstream fq(qmap_path, std::ios::binary);
  std::ofstream fcsv(est_path);
  fcsv << "frame_id,ts,det_valid,method,confidence,est_valid,cx,cy,w,h,"
          "state,source,cost_ms\n";

  rm2027_roi::RoiPipeline pipeline;
  std::vector<std::array<uint8_t, 4>> rois;
  rois.reserve(static_cast<std::size_t>(n));
  std::map<std::string, int> methods;
  int lost = 0;
  int written = 0;
  int idx = 0;
  while (idx < n) {
    cv::Mat frame;
    cap >> frame;
    if (frame.empty()) {
      break;
    }
    rm2027_roi::FrameResult result;
    const auto estimate =
        pipeline.process(frame, idx / fps, &result);
    if (!estimate.valid) {
      ++lost;
    }
    methods[result.detection.valid ? result.detection.method : "NONE"] += 1;

    cv::Mat canvas;
    rm2027_roi::Qpmap qmap;
    if (result.valid_crop) {
      const cv::Rect crop(result.crop.x0, result.crop.y0,
                          result.crop.w, result.crop.h);
      cv::Mat roi_img = frame(crop);
      const int interpolation =
          (rm2027_roi::kCanvasW <= result.crop.w) ? cv::INTER_AREA
                                                  : cv::INTER_LINEAR;
      cv::resize(roi_img, canvas,
                 cv::Size(rm2027_roi::kCanvasW, rm2027_roi::kCanvasH),
                 0, 0, interpolation);
      qmap = result.qmap;
    } else {
      canvas = cv::Mat::zeros(rm2027_roi::kCanvasH, rm2027_roi::kCanvasW,
                              CV_8UC3);
      qmap.data.fill(rm2027_roi::kBgQp);
    }
    cv::Mat yuv;
    cv::cvtColor(canvas, yuv, cv::COLOR_BGR2YUV_I420);
    fyuv.write(reinterpret_cast<const char*>(yuv.data),
               static_cast<std::streamsize>(yuv.total() * yuv.elemSize()));
    fq.write(reinterpret_cast<const char*>(qmap.data.data()),
             static_cast<std::streamsize>(qmap.data.size()));
    rois.push_back(result.valid_crop ? result.roi_normalized
                                     : std::array<uint8_t, 4>{0, 0, 0, 0});

    fcsv << idx << ',' << std::fixed << std::setprecision(4) << idx / fps
         << ',' << (result.detection.valid ? 1 : 0) << ','
         << result.detection.method << ',' << result.detection.confidence
         << ',' << (estimate.valid ? 1 : 0) << ',' << estimate.cx << ','
         << estimate.cy << ',' << estimate.w << ',' << estimate.h << ','
         << estimate.state << ',' << estimate.source << ','
         << estimate.cost_ms << '\n';
    ++written;
    ++idx;
  }
  cap.release();
  fyuv.close();
  fq.close();
  fcsv.close();

  double bitrate = 0.0;
  std::size_t stream_bytes = 0;
  std::size_t packet_count = 0;
  if (!encoder.empty()) {
    std::ostringstream cmd;
    cmd << '"' << encoder << "\" --input \"" << canvas_path.string()
        << "\" --output \"" << stream_path.string() << "\" --qmap \""
        << qmap_path.string() << "\" --csv \"" << enc_csv.string()
        << "\" --fps " << fps << " --frames " << written
        << " --bitrate-kbps " << bitrate_kbps << " --gop " << gop;
    const int rc = std::system(cmd.str().c_str());
    if (rc != 0) {
      std::cerr << "encoder failed with code " << rc << "\n";
      return 3;
    }
    std::ifstream fenc(enc_csv);
    std::string header;
    std::getline(fenc, header);
    const auto cols = split(header, ',');
    int bytes_col = -1;
    for (std::size_t i = 0; i < cols.size(); ++i) {
      if (cols[i] == "bytes") {
        bytes_col = static_cast<int>(i);
      }
    }
    if (bytes_col < 0) {
      std::cerr << "encoder csv has no bytes column\n";
      return 3;
    }
    std::vector<std::size_t> sizes;
    std::string line;
    while (std::getline(fenc, line)) {
      const auto fields = split(line, ',');
      if (static_cast<int>(fields.size()) > bytes_col) {
        sizes.push_back(static_cast<std::size_t>(
            std::stoull(fields[static_cast<std::size_t>(bytes_col)])));
      }
    }
    std::ifstream fs(stream_path, std::ios::binary);
    std::vector<uint8_t> stream((std::istreambuf_iterator<char>(fs)),
                                std::istreambuf_iterator<char>());
    std::size_t total_bytes = 0;
    for (auto s : sizes) {
      total_bytes += s;
    }
    if (sizes.size() < static_cast<std::size_t>(written) ||
        total_bytes != stream.size()) {
      std::cerr << "encoder size mismatch\n";
      return 3;
    }
    std::vector<rm2027_roi::Au> aus;
    std::size_t offset = 0;
    for (int i = 0; i < written; ++i) {
      rm2027_roi::Au au;
      au.frame_id = static_cast<uint16_t>(i);
      au.start = offset;
      au.size = sizes[static_cast<std::size_t>(i)];
      au.roi = rois[static_cast<std::size_t>(i)];
      au.has_roi = true;
      aus.push_back(au);
      offset += au.size;
    }
    const auto packets = rm2027_roi::packetize(stream, aus);
    std::ofstream fp(packets_path, std::ios::binary);
    for (const auto& p : packets) {
      const auto raw = rm2027_roi::encodePacket(p);
      fp.write(reinterpret_cast<const char*>(raw.data()),
               static_cast<std::streamsize>(raw.size()));
    }
    stream_bytes = stream.size();
    packet_count = packets.size();
    const double duration = std::max(1e-6, written / fps);
    bitrate = stream_bytes * 8.0 / duration / 1000.0;
  }

  std::cout << "frames=" << written << " lost=" << lost
            << " methods=";
  for (const auto& kv : methods) {
    std::cout << kv.first << ":" << kv.second << " ";
  }
  std::cout << "\n";
  if (!encoder.empty()) {
    std::cout << "stream=" << stream_bytes << " B bitrate=" << std::fixed
              << std::setprecision(3) << bitrate << " kbps packets="
              << packet_count << "\n";
  }
  std::cout << "wrote " << out_dir << "\n";
  return 0;
}
