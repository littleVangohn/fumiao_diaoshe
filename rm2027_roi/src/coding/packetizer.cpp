#include "rm2027_roi/coding/packetizer.hpp"

#include <algorithm>
#include <cstring>
#include <stdexcept>

namespace rm2027_roi {

std::vector<Packet> packetize(const std::vector<uint8_t>& stream,
                              const std::vector<Au>& aus) {
  std::vector<Packet> packets;
  if (aus.empty()) {
    return packets;
  }
  std::size_t total = 0;
  for (const auto& au : aus) {
    total += au.size;
  }
  if (total != stream.size()) {
    throw std::runtime_error("AU sizes do not match stream size");
  }
  std::vector<std::size_t> starts;
  starts.reserve(aus.size());
  for (const auto& au : aus) {
    starts.push_back(au.start);
  }
  std::size_t pos = 0;
  uint16_t seq = 0;
  while (pos < stream.size()) {
    std::size_t au_idx =
        static_cast<std::size_t>(std::upper_bound(starts.begin(), starts.end(),
                                                  pos) - starts.begin());
    au_idx = au_idx == 0 ? 0 : std::min(aus.size() - 1, au_idx - 1);
    const Au& au = aus[au_idx];
    std::size_t end = std::min(pos + kMaxPayload, stream.size());
    auto si_it = std::lower_bound(starts.begin(), starts.end(), pos);
    const std::size_t si =
        static_cast<std::size_t>(si_it - starts.begin());
    bool has_roi = false;
    std::array<uint8_t, kRoiBytes> roi{};
    uint16_t frame_id = au.frame_id;
    uint8_t flags = 0;
    if (si < aus.size() && starts[si] < end) {
      const Au& new_au = aus[si];
      const std::size_t si2 = si + 1;
      if (si2 < aus.size() && starts[si2] < end) {
        end = starts[si2];
      }
      if (starts[si] == pos) {
        flags |= kFlagAuStart;
      }
      roi = new_au.roi;
      has_roi = new_au.has_roi;
      frame_id = new_au.frame_id;
      flags |= kFlagRoi;
      end = std::min(end, pos + kMaxPayload - kRoiBytes);
      if (end <= starts[si]) {
        has_roi = false;
        roi = {};
        frame_id = au.frame_id;
        flags = 0;
        end = starts[si];
      }
    }
    if (end <= pos) {
      throw std::runtime_error("empty chunk");
    }
    Packet p;
    p.seq = seq++;
    p.frame_id = frame_id;
    p.flags = flags;
    if (has_roi) {
      p.payload.insert(p.payload.end(), roi.begin(), roi.end());
      p.has_roi = true;
      p.roi = roi;
    }
    p.payload.insert(p.payload.end(), stream.begin() + static_cast<long>(pos),
                     stream.begin() + static_cast<long>(end));
    packets.push_back(std::move(p));
    pos = end;
  }
  return packets;
}

std::pair<std::vector<uint8_t>,
          std::map<uint16_t, std::array<uint8_t, kRoiBytes>>>
reassemble(const std::vector<Packet>& packets) {
  std::vector<Packet> sorted = packets;
  std::sort(sorted.begin(), sorted.end(),
            [](const Packet& a, const Packet& b) { return a.seq < b.seq; });
  std::vector<uint8_t> stream;
  std::map<uint16_t, std::array<uint8_t, kRoiBytes>> rois;
  for (const auto& p : sorted) {
    const std::size_t skip = (p.flags & kFlagRoi) ? kRoiBytes : 0;
    if (skip > p.payload.size()) {
      continue;
    }
    stream.insert(stream.end(), p.payload.begin() + static_cast<long>(skip),
                  p.payload.end());
    if ((p.flags & kFlagRoi) && p.has_roi) {
      rois[p.frame_id] = p.roi;
    }
  }
  return {stream, rois};
}

std::vector<uint8_t> encodePacket(const Packet& packet) {
  std::vector<uint8_t> raw(kPacketBytes, 0);
  const uint16_t valid_len = static_cast<uint16_t>(packet.payload.size());
  raw[0] = static_cast<uint8_t>(packet.seq & 0xFF);
  raw[1] = static_cast<uint8_t>((packet.seq >> 8) & 0xFF);
  raw[2] = static_cast<uint8_t>(packet.frame_id & 0xFF);
  raw[3] = static_cast<uint8_t>((packet.frame_id >> 8) & 0xFF);
  raw[4] = static_cast<uint8_t>(valid_len & 0xFF);
  raw[5] = static_cast<uint8_t>((valid_len >> 8) & 0xFF);
  raw[6] = packet.flags;
  raw[7] = 0;
  std::copy(packet.payload.begin(), packet.payload.end(),
            raw.begin() + kHeaderBytes);
  return raw;
}

Packet decodePacket(const uint8_t* data, std::size_t len) {
  if (len < kHeaderBytes) {
    throw std::runtime_error("packet shorter than header");
  }
  Packet p;
  p.seq = static_cast<uint16_t>(data[0] | (data[1] << 8));
  p.frame_id = static_cast<uint16_t>(data[2] | (data[3] << 8));
  const uint16_t valid_len = static_cast<uint16_t>(data[4] | (data[5] << 8));
  p.flags = data[6];
  if (valid_len > kMaxPayload || kHeaderBytes + valid_len > len) {
    throw std::runtime_error("invalid packet length");
  }
  p.payload.assign(data + kHeaderBytes, data + kHeaderBytes + valid_len);
  if (p.flags & kFlagRoi && p.payload.size() >= kRoiBytes) {
    std::copy(p.payload.begin(), p.payload.begin() + kRoiBytes,
              p.roi.begin());
    p.has_roi = true;
  }
  return p;
}

}  // namespace rm2027_roi
