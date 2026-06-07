#pragma once

#include <cstdint>
#include <string>
#include <vector>

extern "C" {
#include <libavcodec/packet.h>
#include <libavformat/avformat.h>
}

namespace SpatialMP4 {

struct PacketMuxerResult {
  int rgb_packets = 0;
  int depth_packets = 0;
  int pose_packets = 0;
};

class PacketMuxer {
 public:
  PacketMuxer() = default;
  PacketMuxerResult Mux(const std::string& packet_dir, const std::string& output_mp4);
  PacketMuxerResult MuxRgbPreview(const std::string& packet_dir, const std::string& output_mp4);
};

}  // namespace SpatialMP4
