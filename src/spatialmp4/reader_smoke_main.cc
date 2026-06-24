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

    // Depth sanity: print the parsed depth intrinsics (sourced from the ICAM
    // box, which the patched mov demuxer must surface for FFV1 tracks too) and
    // the metric depth range of the first frame, confirming FFV1 depth decodes
    // to real metres rather than garbage.
    const SpatialML::camera_intrinsics di = reader.GetDepthIntrinsics();
    double dmin = 0.0, dmax = 0.0;
    cv::minMaxLoc(depth.depth, &dmin, &dmax);
    std::cout << "depth intrinsics fx=" << di.fx << " fy=" << di.fy << " cx=" << di.cx << " cy=" << di.cy
              << " | depth metres min=" << dmin << " max=" << dmax << "\n";

    // Multi-track report: confirm the reader classifies controller / hand /
    // input mett tracks by metadata, not just the legacy single head pose.
    const std::vector<std::string> tracks = reader.ListTimedMetadataTracks();
    std::cout << "timed metadata tracks (" << tracks.size() << "):\n";
    for (const std::string& track_id : tracks) {
      std::cout << "  - " << track_id << ": rigid_pose=" << reader.GetRigidPoseFrames(track_id).size()
                << " hand_joints=" << reader.GetHandJointFrames(track_id).size()
                << " body_joints=" << reader.GetBodyJointFrames(track_id).size()
                << " controller_input=" << reader.GetControllerInputFrames(track_id).size() << "\n";
    }
  } catch (const std::exception& error) {
    std::cerr << "spatialmp4_reader_smoke failed: " << error.what() << "\n";
    return 1;
  }

  return 0;
}
