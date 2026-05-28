#include "spatialmp4/reader.h"

#include <iostream>
#include <stdexcept>

int main(int argc, char** argv) {
  if (argc != 2) {
    std::cerr << "Usage: spatialmp4_reader_smoke <capture.mp4>\n";
    return 2;
  }

  try {
    SpatialML::Reader reader(argv[1], "warning");
    if (!reader.HasRGB() || !reader.HasDepth() || !reader.HasPose()) {
      throw std::runtime_error("missing RGB, depth, or pose stream");
    }
    if (reader.GetRgbWidth() <= 0 || reader.GetRgbHeight() <= 0 || reader.GetDepthWidth() <= 0 ||
        reader.GetDepthHeight() <= 0 || reader.GetDepthFPS() <= 0.0f) {
      throw std::runtime_error("invalid RGB/depth dimensions or depth frame rate");
    }
    if (reader.GetPoseFrames().empty()) {
      throw std::runtime_error("pose track contains no samples");
    }

    reader.SetReadMode(SpatialML::Reader::RGB_ONLY);
    SpatialML::rgb_frame rgb;
    reader.Load(rgb);
    if (rgb.left_rgb.empty() || rgb.right_rgb.empty()) {
      throw std::runtime_error("failed to decode the first RGB stereo frame");
    }

    reader.Reset();
    reader.SetReadMode(SpatialML::Reader::DEPTH_ONLY);
    SpatialML::depth_frame depth;
    reader.Load(depth, true);
    if (depth.depth.empty()) {
      throw std::runtime_error("failed to read the first depth frame");
    }

    std::cout << "reader loaded RGB " << rgb.left_rgb.cols << "x" << rgb.left_rgb.rows << " per eye, depth "
              << depth.depth.cols << "x" << depth.depth.rows << ", pose samples " << reader.GetPoseFrames().size()
              << "\n";
  } catch (const std::exception& error) {
    std::cerr << "spatialmp4_reader_smoke failed: " << error.what() << "\n";
    return 1;
  }

  return 0;
}
