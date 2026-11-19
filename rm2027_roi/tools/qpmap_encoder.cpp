// Frozen baseline encoder: 320x240, QPMAP BG+10 / FG-5 / NEUT 0,
// libx264 ABR, B=0, lookahead=0, AQ=2, mbtree=0, single thread.
//
// Input : raw I420 320x240 frames + 300-byte QPMAP per frame (20x15 blocks)
// Output: Annex-B H.264 stream + per-frame CSV
//
// Build: bash build_encoder.sh
#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iostream>
#include <map>
#include <stdexcept>
#include <string>
#include <vector>

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavutil/error.h>
#include <libavutil/frame.h>
#include <libavutil/opt.h>
#include <libavutil/rational.h>
}

namespace {

using Clock = std::chrono::steady_clock;
constexpr int kWidth = 320;
constexpr int kHeight = 240;
constexpr int kMbWidth = 20;
constexpr int kMbHeight = 15;
constexpr int kMapBytes = kMbWidth * kMbHeight;  // 300

std::string av_error(int code) {
    char buffer[AV_ERROR_MAX_STRING_SIZE]{};
    av_strerror(code, buffer, sizeof(buffer));
    return buffer;
}

struct Options {
    std::string input;
    std::string output;
    std::string qmap;
    std::string csv;
    int fps = 30;
    int frames = 600;
    int bitrate_kbps = 88;
    int gop = 60;
};

int parse_int(const char* value, const char* name) {
    try {
        return std::stoi(value);
    } catch (...) {
        throw std::runtime_error(std::string("invalid integer for ") + name);
    }
}

Options parse_options(int argc, char** argv) {
    Options o;
    for (int i = 1; i < argc; ++i) {
        const std::string key = argv[i];
        auto value = [&]() -> const char* {
            if (++i >= argc) throw std::runtime_error("missing value for " + key);
            return argv[i];
        };
        if (key == "--input") o.input = value();
        else if (key == "--output") o.output = value();
        else if (key == "--qmap") o.qmap = value();
        else if (key == "--csv") o.csv = value();
        else if (key == "--fps") o.fps = parse_int(value(), "fps");
        else if (key == "--frames") o.frames = parse_int(value(), "frames");
        else if (key == "--bitrate-kbps") o.bitrate_kbps = parse_int(value(), "bitrate-kbps");
        else if (key == "--gop") o.gop = parse_int(value(), "gop");
        else throw std::runtime_error("unknown option: " + key);
    }
    if (o.input.empty() || o.output.empty() || o.qmap.empty() || o.csv.empty()) {
        throw std::runtime_error("--input, --output, --qmap and --csv are required");
    }
    if (o.frames <= 0 || o.fps <= 0 || o.bitrate_kbps <= 0 || o.gop <= 0) {
        throw std::runtime_error("fps/frames/bitrate/gop must be positive");
    }
    return o;
}

void read_plane(std::ifstream& input, uint8_t* dst, int linesize, int width, int height) {
    for (int y = 0; y < height; ++y) {
        input.read(reinterpret_cast<char*>(dst + y * linesize), width);
        if (input.gcount() != width) {
            throw std::runtime_error("unexpected end of raw YUV input");
        }
    }
}

void attach_qmap(AVFrame* frame, const std::vector<int8_t>& map) {
    if (static_cast<int>(map.size()) != kMapBytes) {
        throw std::runtime_error("QPMAP must be exactly 300 blocks");
    }
    AVFrameSideData* side = av_frame_new_side_data(
        frame, AV_FRAME_DATA_REGIONS_OF_INTEREST,
        map.size() * sizeof(AVRegionOfInterest));
    if (!side) throw std::runtime_error("cannot allocate ROI side data");
    auto* regions = reinterpret_cast<AVRegionOfInterest*>(side->data);
    for (int my = 0; my < kMbHeight; ++my) {
        for (int mx = 0; mx < kMbWidth; ++mx) {
            const int index = my * kMbWidth + mx;
            AVRegionOfInterest& roi = regions[index];
            std::memset(&roi, 0, sizeof(roi));
            roi.self_size = sizeof(roi);
            roi.left = mx * 16;
            roi.right = std::min(kWidth, (mx + 1) * 16);
            roi.top = my * 16;
            roi.bottom = std::min(kHeight, (my + 1) * 16);
            roi.qoffset = AVRational{static_cast<int>(map[index]), 51};
        }
    }
}

}  // namespace

int main(int argc, char** argv) {
    try {
        const Options o = parse_options(argc, argv);
        std::ifstream input(o.input, std::ios::binary);
        if (!input) throw std::runtime_error("cannot open input: " + o.input);
        std::ifstream qmap_input(o.qmap, std::ios::binary);
        if (!qmap_input) throw std::runtime_error("cannot open qmap: " + o.qmap);
        std::ofstream output(o.output, std::ios::binary);
        std::ofstream csv(o.csv);
        if (!output || !csv) throw std::runtime_error("cannot create output files");

        const AVCodec* codec = avcodec_find_encoder_by_name("libx264");
        if (!codec) throw std::runtime_error("FFmpeg libx264 encoder is unavailable");
        AVCodecContext* context = avcodec_alloc_context3(codec);
        if (!context) throw std::runtime_error("cannot allocate codec context");
        context->width = kWidth;
        context->height = kHeight;
        context->pix_fmt = AV_PIX_FMT_YUV420P;
        context->time_base = AVRational{1, o.fps};
        context->framerate = AVRational{o.fps, 1};
        context->gop_size = o.gop;
        context->max_b_frames = 0;
        context->refs = 2;
        context->thread_count = 1;            // deterministic frozen baseline
        context->flags |= AV_CODEC_FLAG_LOW_DELAY;
        av_opt_set(context->priv_data, "preset", "faster", 0);
        const std::string x264_params =
            "rc-lookahead=0:sync-lookahead=0:sliced-threads=1:scenecut=0"
            ":repeat-headers=1:aq-mode=2:aq-strength=1.0:mbtree=0:force-cfr=1";
        context->bit_rate = static_cast<int64_t>(o.bitrate_kbps) * 1000;
        context->rc_max_rate = context->bit_rate;
        context->rc_buffer_size = std::max(1000, o.bitrate_kbps * 100);
        const std::string vbv =
            "vbv-maxrate=" + std::to_string(o.bitrate_kbps) +
            ":vbv-bufsize=" + std::to_string(std::max(1, o.bitrate_kbps / 10));
        av_opt_set(context->priv_data, "x264-params", (x264_params + ":" + vbv).c_str(), 0);

        int rc = avcodec_open2(context, codec, nullptr);
        if (rc < 0) throw std::runtime_error("avcodec_open2: " + av_error(rc));

        AVFrame* frame = av_frame_alloc();
        AVPacket* packet = av_packet_alloc();
        if (!frame || !packet) throw std::runtime_error("cannot allocate AVFrame/AVPacket");
        frame->format = context->pix_fmt;
        frame->width = kWidth;
        frame->height = kHeight;
        rc = av_frame_get_buffer(frame, 32);
        if (rc < 0) throw std::runtime_error("av_frame_get_buffer: " + av_error(rc));

        std::vector<int8_t> qmap(kMapBytes);
        std::vector<Clock::time_point> submitted(o.frames);
        std::map<int64_t, size_t> bytes_by_pts;
        std::map<int64_t, double> ready_us_by_pts;
        std::map<int64_t, int> emitted_after_input;
        int submissions = 0;
        const auto run_start = Clock::now();

        auto drain = [&]() {
            while (true) {
                int result = avcodec_receive_packet(context, packet);
                if (result == AVERROR(EAGAIN) || result == AVERROR_EOF) return;
                if (result < 0) throw std::runtime_error("avcodec_receive_packet: " + av_error(result));
                output.write(reinterpret_cast<const char*>(packet->data), packet->size);
                bytes_by_pts[packet->pts] += static_cast<size_t>(packet->size);
                if (packet->pts >= 0 && packet->pts < o.frames) {
                    ready_us_by_pts[packet->pts] = std::chrono::duration<double, std::micro>(
                        Clock::now() - submitted[packet->pts]).count();
                    emitted_after_input[packet->pts] = submissions;
                }
                av_packet_unref(packet);
            }
        };

        for (int index = 0; index < o.frames; ++index) {
            rc = av_frame_make_writable(frame);
            if (rc < 0) throw std::runtime_error("av_frame_make_writable: " + av_error(rc));
            read_plane(input, frame->data[0], frame->linesize[0], kWidth, kHeight);
            read_plane(input, frame->data[1], frame->linesize[1], kWidth / 2, kHeight / 2);
            read_plane(input, frame->data[2], frame->linesize[2], kWidth / 2, kHeight / 2);
            qmap_input.read(reinterpret_cast<char*>(qmap.data()), kMapBytes);
            if (qmap_input.gcount() != kMapBytes) {
                throw std::runtime_error("unexpected end of QPMAP at frame " + std::to_string(index));
            }
            frame->pts = index;
            av_frame_remove_side_data(frame, AV_FRAME_DATA_REGIONS_OF_INTEREST);
            attach_qmap(frame, qmap);
            submitted[index] = Clock::now();
            rc = avcodec_send_frame(context, frame);
            if (rc < 0) throw std::runtime_error("avcodec_send_frame: " + av_error(rc));
            submissions = index + 1;
            drain();
        }
        rc = avcodec_send_frame(context, nullptr);
        if (rc < 0) throw std::runtime_error("encoder flush: " + av_error(rc));
        drain();

        const double wall_seconds =
            std::chrono::duration<double>(Clock::now() - run_start).count();
        submissions = o.frames;
        csv << "frame,pts,bytes,encode_ready_us,emitted_after_input\n";
        for (int index = 0; index < o.frames; ++index) {
            csv << index << ',' << index << ',' << bytes_by_pts[index] << ',';
            auto it = ready_us_by_pts.find(index);
            if (it != ready_us_by_pts.end()) csv << it->second;
            csv << ',' << emitted_after_input[index] << '\n';
        }
        std::cerr << "{\"frames\":" << o.frames
                  << ",\"wall_seconds\":" << wall_seconds
                  << ",\"encode_fps\":" << (o.frames / wall_seconds)
                  << ",\"bytes\":" << output.tellp()
                  << ",\"threads\":1,\"qmap\":true}\n";

        av_packet_free(&packet);
        av_frame_free(&frame);
        avcodec_free_context(&context);
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "qpmap_encoder: " << error.what() << '\n';
        return 2;
    }
}
