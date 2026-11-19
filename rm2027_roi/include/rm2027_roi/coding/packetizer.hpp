#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <map>
#include <utility>
#include <vector>

namespace rm2027_roi {

constexpr int kPacketBytes = 300;
constexpr int kHeaderBytes = 8;
constexpr int kMaxPayload = kPacketBytes - kHeaderBytes;
constexpr int kRoiBytes = 4;
constexpr uint8_t kFlagRoi = 1;
constexpr uint8_t kFlagAuStart = 2;

struct Packet {
  uint16_t seq = 0;
  uint16_t frame_id = 0;
  uint8_t flags = 0;
  std::vector<uint8_t> payload;
  std::array<uint8_t, kRoiBytes> roi{};
  bool has_roi = false;
};

struct Au {
  uint16_t frame_id = 0;
  std::size_t start = 0;
  std::size_t size = 0;
  std::array<uint8_t, kRoiBytes> roi{};
  bool has_roi = false;
};

std::vector<Packet> packetize(const std::vector<uint8_t>& stream,
                              const std::vector<Au>& aus);
std::pair<std::vector<uint8_t>, std::map<uint16_t, std::array<uint8_t, kRoiBytes>>>
reassemble(const std::vector<Packet>& packets);
std::vector<uint8_t> encodePacket(const Packet& packet);
Packet decodePacket(const uint8_t* data, std::size_t len);

}  // namespace rm2027_roi
