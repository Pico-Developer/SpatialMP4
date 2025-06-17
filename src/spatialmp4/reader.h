/*
 * Copyright (c) 2025 Bytedance Ltd. and/or its affiliates
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#pragma once
#include <iostream>
#include <vector>
#include <string>
#include <unordered_map>
#include <opencv2/opencv.hpp>
#include "utilities/SyncPose.hpp"
#include "spatialmp4/data_types.h"

extern "C" {
#include <libavformat/avformat.h>
#include <libavcodec/avcodec.h>
#include <libswscale/swscale.h>
#include <libavutil/pixfmt.h>
#include <libavutil/pixdesc.h>
}

#if defined(__APPLE__) && defined(__clang__)
#include <filesystem>
namespace fs = std::__fs::filesystem;
#else
#include <experimental/filesystem>
namespace fs = std::experimental::filesystem;
#endif

namespace SpatialML {

/**
 * Random access video reader.
 * This class is used to read a video file in random access mode.
 */
class RandomAccessVideoReader {
 public:
  RandomAccessVideoReader() = default;
  ~RandomAccessVideoReader();
  bool Open(const std::string& filename, bool isSpatialMP4 = false);
  AVFrame* GetFrame(int64_t frame_number);
  int64_t GetFrameCount() const;
  void Debug();

 private:
  void build_keyframe_index();
  bool seek_to_keyframe(int64_t timestamp);
  AVFrame* process_frame(AVFrame* frame);

  AVFormatContext* fmt_ctx_ = nullptr;
  AVCodecContext* codec_ctx_ = nullptr;
  const AVCodec* codec_ = nullptr;
  int video_stream_index_ = -1;
  std::vector<int64_t> keyframes_;
  std::vector<int64_t> allframes_;
  int64_t prev_seek_pts_ = 0;
};

/**
 * SpatialMP4 Reader.
 * This class is used to read a SpatialMP4 file.
 * The file format is described in the SpatialMP4 format specification.
 */
class Reader {
 public:
  enum ReadMode {
    RGB_ONLY,
    DEPTH_ONLY,
    DEPTH_FIRST,
  };
  static std::string ReadMode2String(ReadMode mode) {
    switch (mode) {
      case RGB_ONLY:
        return "RGB_ONLY";
      case DEPTH_ONLY:
        return "DEPTH_ONLY";
      case DEPTH_FIRST:
        return "DEPTH_FIRST";
      default:
        return "UNKNOWN";
    }
  }

  Reader(const std::string& filename);
  ~Reader();

  bool HasRGB() const { return has_rgb_; }
  bool HasDepth() const { return has_depth_; }
  bool HasPose() const { return has_pose_; }
  bool HasAudio() const { return has_audio_; }
  bool HasDisparity() const { return has_disparity_; }

  uint64_t GetStartTimestamp() const { return start_timestamp_; }
  float GetDuration() const { return duration_; }
  float GetRgbFPS() const { return rgb_fps_; }
  int GetRgbWidth() const { return rgb_width_; }
  int GetRgbHeight() const { return rgb_height_; }
  camera_intrinsics GetRgbIntrinsicsLeft() const { return rgb_intrinsics_left_; }
  camera_extrinsics GetRgbExtrinsicsLeft() const { return rgb_extrinsics_left_; }
  camera_intrinsics GetRgbIntrinsicsRight() const { return rgb_intrinsics_right_; }
  camera_extrinsics GetRgbExtrinsicsRight() const { return rgb_extrinsics_right_; }
  float GetDepthFPS() const { return depth_fps_; }
  int GetDepthWidth() const { return depth_width_; }
  int GetDepthHeight() const { return depth_height_; }
  camera_intrinsics GetDepthIntrinsics() const { return depth_intrinsics_; }
  camera_extrinsics GetDepthExtrinsics() const { return depth_extrinsics_; }
  std::vector<pose_frame> GetPoseFrames() const { return pose_frames_.getAllPose(); }
  int GetRgbKeyframeIndex() const { return keyframe_rgb_idx_; }
  int GetDepthKeyframeIndex() const { return keyframe_depth_idx_; }
  bool IsRgbDistorted() const { return is_rgb_distorted_; }
  std::string GetRgbDistortionModel() const { return rgb_distortion_model_; }
  std::string GetRgbDistortionParamsLeft() const { return rgb_distortion_params_left_; }
  std::string GetRgbDistortionParamsRight() const { return rgb_distortion_params_right_; }
  std::string GetDepthDistortionModel() const { return depth_distortion_model_; }
  std::string GetDepthDistortionParams() const { return depth_distortion_params_; }
  uint64_t GetRgbTimebase() const { return rgb_timebase_; }
  uint64_t GetDepthTimebase() const { return depth_timebase_; }

  void SetReadMode(ReadMode mode) { read_mode_ = mode; }
  bool HasNext() const;
  void Load(rgb_frame& rgb_frame);
  void Load(depth_frame& depth_frame);
  void Load(rgb_frame& rgb_frame, depth_frame& depth_frame);
  void Reset();
  int GetIndex() const;

 protected:
  void LoadAllPoseData(int frame_id);
  void ParseDepthFrame(const AVPacket& pkt, depth_frame& depth_frame);
  void ParseRgbFrame(const AVPacket& pkt, rgb_frame& rgb_frame, bool skip = false);

  bool SeekToRgbKeyframe(int64_t timestamp);

 private:
  std::string filename_;
  ReadMode read_mode_;
  AVFormatContext* pFormatCtx_;
  AVPacket current_packet_;

  AVCodecContext* rgb_codec_ctx_;
  const AVCodec* rgb_codec_;
  AVFrame* rgb_frame_;

  // media data info
  bool has_rgb_;
  int rgb_frame_id_;
  bool has_depth_;
  int depth_frame_id_;
  bool has_pose_;
  int pose_frame_id_;
  bool has_audio_;
  int audio_frame_id_;
  int audio_2_frame_id_;
  bool has_disparity_;
  int disparity_frame_id_;
  uint64_t rgb_timebase_;
  uint64_t depth_timebase_;
  uint64_t start_timestamp_;
  float duration_;
  float rgb_fps_;
  int rgb_width_;
  int rgb_height_;
  float depth_fps_;
  int depth_width_;
  int depth_height_;
  camera_intrinsics rgb_intrinsics_left_;
  camera_extrinsics rgb_extrinsics_left_;
  camera_intrinsics rgb_intrinsics_right_;
  camera_extrinsics rgb_extrinsics_right_;
  camera_intrinsics depth_intrinsics_;
  camera_extrinsics depth_extrinsics_;
  bool is_rgb_distorted_;
  std::string rgb_distortion_model_;
  std::string rgb_distortion_params_left_;
  std::string rgb_distortion_params_right_;
  std::string depth_distortion_model_;
  std::string depth_distortion_params_;

  std::vector<int64_t> keyframes_rgb;
  std::vector<int64_t> allframes_rgb;
  std::vector<int64_t> keyframes_depth;
  int keyframe_rgb_idx_;
  int allframe_rgb_idx_;
  int keyframe_depth_idx_;

  // media data
  Utilities::SynchronizedQueue<pose_frame> pose_frames_;
  double find_pose_distance_ = 0.001;
  Utilities::SynchronizedQueue<int64_t> rgb_frame_pts_queue_;
};

}  // namespace SpatialML
