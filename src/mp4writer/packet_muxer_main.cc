#include "mp4writer/packet_muxer.h"

#include <exception>
#include <iostream>

int main(int argc, char** argv) {
  if (argc != 3) {
    std::cerr << "Usage: spatialmp4_packet_muxer <packet_dir> <output.mp4>\n";
    return 2;
  }

  try {
    SpatialMP4::PacketMuxer muxer;
    const auto result = muxer.Mux(argv[1], argv[2]);
    std::cout << "wrote " << argv[2] << " with " << result.rgb_packets << " RGB packets, "
              << result.depth_packets << " depth packets, and " << result.pose_packets << " pose packets\n";
  } catch (const std::exception& error) {
    std::cerr << "spatialmp4_packet_muxer failed: " << error.what() << "\n";
    return 1;
  }

  return 0;
}
