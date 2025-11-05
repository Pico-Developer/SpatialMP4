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

#include "spatialmp4/reader.h"
#include "spatialmp4/utils.h"
#include <iostream>
#include <unordered_map>
#include <thread>
#include <chrono>
#include <spdlog/spdlog.h>

namespace SpatialML {

/******************** RandomAccessVideoReader ********************/

RandomAccessVideoReader::~RandomAccessVideoReader() {
  spdlog::debug("RandomAccessVideoReader::~RandomAccessVideoReader()");
  if (codec_ctx_) avcodec_free_context(&codec_ctx_);
  if (fmt_ctx_) avformat_close_input(&fmt_ctx_);
  spdlog::debug("RandomAccessVideoReader::~RandomAccessVideoReader() end");
}

bool RandomAccessVideoReader::Open(const std::string& filename, bool isSpatialMP4) {
  // init ffmpeg context
  if (avformat_open_input(&fmt_ctx_, filename.c_str(), nullptr, nullptr) != 0) {
    return false;
  }

  if (avformat_find_stream_info(fmt_ctx_, nullptr) < 0) {
    return false;
  }

  if (isSpatialMP4) {
    // find video stream
    auto stream_type_info = GetStreamTypeInfo(fmt_ctx_);
    for (auto& pair : stream_type_info) {
      int stream_index = pair.first;
      StreamType stream_type = pair.second;
      if (stream_type == MEDIA_TYPE_RGB) {
        video_stream_index_ = stream_index;
        break;
      }
    }
  } else {
    video_stream_index_ = av_find_best_stream(fmt_ctx_, AVMEDIA_TYPE_VIDEO, -1, -1, &codec_, 0);
    if (video_stream_index_ < 0) {
      return false;
    }
  }
  spdlog::info("video_stream_index: {}", video_stream_index_);

  // init codec and codec context
  codec_ = avcodec_find_decoder(fmt_ctx_->streams[video_stream_index_]->codecpar->codec_id);
  if (!codec_) {
    throw std::runtime_error("RGB codec not found");
  }
  codec_ctx_ = avcodec_alloc_context3(codec_);
  if (!codec_ctx_) {
    throw std::runtime_error("Could not allocate codec context");
  }
  if (avcodec_parameters_to_context(codec_ctx_, fmt_ctx_->streams[video_stream_index_]->codecpar) < 0) {
    throw std::runtime_error("Could not copy codec parameters");
  }
  if (avcodec_open2(codec_ctx_, codec_, NULL) < 0) {
    return false;
  }
  spdlog::debug("codec has B frames: {}", codec_ctx_->has_b_frames);

  // build keyframe index
  build_keyframe_index();
  return true;
}

AVFrame* RandomAccessVideoReader::GetFrame(int64_t frame_number) {
  if (frame_number >= allframes_.size()) {
    spdlog::error("frame_number({}) is out of range({})", frame_number, allframes_.size());
    return nullptr;
  }
  int64_t target_pts = allframes_[frame_number];
  // find the nearest keyframe
  auto compare = [](int64_t a, int64_t b) { return a < b; };
  auto it = std::upper_bound(keyframes_.begin(), keyframes_.end(), target_pts, compare);
  if (it != keyframes_.begin()) --it;
  // std::cout << "keyframe_pts: " << *it << ", target_pts: " << target_pts << std::endl;

  const int64_t keyframe_pts = *it;
  seek_to_keyframe(keyframe_pts);

  AVFrame* result = nullptr;
  AVPacket pkt;
  AVFrame* frame = av_frame_alloc();

  bool is_first_run = true;
  while (is_first_run || pkt.pts <= target_pts) {
    is_first_run = false;
    int ret = av_read_frame(fmt_ctx_, &pkt);
    if (ret < 0) break;
    if (pkt.stream_index != video_stream_index_) {
      av_packet_unref(&pkt);
      continue;
    }
    ret = avcodec_send_packet(codec_ctx_, &pkt);
    if (ret < 0) {
      spdlog::warn("avcodec_send_packet failed: {}", FFmpegErrorString(ret));
      av_packet_unref(&pkt);
      return result;
    }
    ret = avcodec_receive_frame(codec_ctx_, frame);
    if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF) {
      spdlog::warn("avcodec_receive_frame failed: {}", FFmpegErrorString(ret));
      av_packet_unref(&pkt);
      break;
    }

    // std::cout << "pkt.pts: " << pkt.pts << ", target_pts: " << target_pts << std::endl;
    if (pkt.pts == target_pts) {
      result = process_frame(frame);
      double result_timestamp = result->pts * av_q2d(fmt_ctx_->streams[video_stream_index_]->time_base);
      spdlog::debug("decode target_pts: {}, packet.pts: {}, frame.pts: {}, result_timestamp: {:.4f}", target_pts,
                    pkt.pts, frame->pts, result_timestamp);
      break;
    }
    av_packet_unref(&pkt);
  }
  av_frame_free(&frame);
  return result;
}

int64_t RandomAccessVideoReader::GetFrameCount() const { return fmt_ctx_->streams[video_stream_index_]->nb_frames; }

void RandomAccessVideoReader::Debug() {
  av_seek_frame(fmt_ctx_, video_stream_index_, 0, AVSEEK_FLAG_BACKWARD);
  AVPacket pkt;
  int frame_count = 0;
  AVFrame* frame = av_frame_alloc();
  while (av_read_frame(fmt_ctx_, &pkt) >= 0) {
    if (pkt.stream_index == video_stream_index_) {
      double pkt_timestamp = pkt.pts * av_q2d(fmt_ctx_->streams[video_stream_index_]->time_base);

      int ret = avcodec_send_packet(codec_ctx_, &pkt);
      if (ret < 0) {
        spdlog::warn("avcodec_send_packet failed: {}", FFmpegErrorString(ret));
        av_packet_unref(&pkt);
        continue;
      }

      ret = avcodec_receive_frame(codec_ctx_, frame);
      if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF) {
        spdlog::warn("avcodec_receive_frame failed: {}", FFmpegErrorString(ret));
        continue;
      }
      spdlog::debug(
          "packet pts: {}, packet timestamp: {:.4f}, stream_index: {}, frame_count: {}, is_keyframe: {}, frame pts: {}",
          pkt.pts, pkt_timestamp, pkt.stream_index, frame_count, pkt.flags & AV_PKT_FLAG_KEY, frame->pts);
      av_packet_unref(&pkt);
      frame_count++;
    }
  }
  av_frame_free(&frame);
}

void RandomAccessVideoReader::build_keyframe_index() {
  AVPacket pkt;
  while (av_read_frame(fmt_ctx_, &pkt) >= 0) {
    if (pkt.stream_index == video_stream_index_) {
      allframes_.push_back(pkt.pts);
      if (pkt.flags & AV_PKT_FLAG_KEY) {
        keyframes_.push_back(pkt.pts);
      }
    }
    av_packet_unref(&pkt);
  }
  spdlog::info("keyframes size: {}", keyframes_.size());
  spdlog::info("allframes size: {}", allframes_.size());
}

bool RandomAccessVideoReader::seek_to_keyframe(int64_t target_pts) {
  if (target_pts == prev_seek_pts_) {
    return false;
  }
  int ret = av_seek_frame(fmt_ctx_, video_stream_index_, target_pts, AVSEEK_FLAG_BACKWARD);
  if (ret < 0) {
    spdlog::error("seek to keyframe failed: {}", SpatialML::FFmpegErrorString(ret));
    return false;
  }
  avcodec_flush_buffers(codec_ctx_);
  prev_seek_pts_ = target_pts;
  spdlog::debug("seek to keyframe_pts: {}", target_pts);
  return true;
}

AVFrame* RandomAccessVideoReader::process_frame(AVFrame* frame) {
  // convert to RGB format
  SwsContext* sws_ctx = sws_getContext(frame->width, frame->height, (AVPixelFormat)frame->format, frame->width,
                                       frame->height, AV_PIX_FMT_RGB24, SWS_BILINEAR, nullptr, nullptr, nullptr);

  AVFrame* rgb_frame = av_frame_alloc();
  rgb_frame->format = AV_PIX_FMT_RGB24;
  rgb_frame->width = frame->width;
  rgb_frame->height = frame->height;
  if (av_frame_get_buffer(rgb_frame, 0) < 0) {
    std::cerr << "Failed to allocate frame buffer" << std::endl;
    av_frame_free(&rgb_frame);
    sws_freeContext(sws_ctx);
    return NULL;
  }
  sws_scale(sws_ctx, frame->data, frame->linesize, 0, frame->height, rgb_frame->data, rgb_frame->linesize);

  sws_freeContext(sws_ctx);
  rgb_frame->pts = frame->pts;
  return rgb_frame;
}

/******************** Reader ********************/

void SetFFmpegLogLevel(const std::string& log_level) {
  static const std::unordered_map<std::string, int> log_level_map = {
    {"quiet", AV_LOG_QUIET},
    {"panic", AV_LOG_PANIC},
    {"fatal", AV_LOG_FATAL},
    {"error", AV_LOG_ERROR},
    {"warning", AV_LOG_WARNING},
    {"info", AV_LOG_INFO},
    {"verbose", AV_LOG_VERBOSE},
    {"debug", AV_LOG_DEBUG},
    {"trace", AV_LOG_TRACE}
  };

  auto it = log_level_map.find(log_level);
  if (it != log_level_map.end()) {
    av_log_set_level(it->second);
  } else {
    spdlog::warn("Invalid FFmpeg log level: '{}'. Using 'warning' as default.", log_level);
    av_log_set_level(AV_LOG_WARNING);
  }
}

Reader::Reader(const std::string& filename, const std::string& log_level)
    : filename_(filename),
      log_level_(log_level),
      read_mode_(ReadMode::DEPTH_FIRST),
      pFormatCtx_(NULL),
      has_rgb_(false),
      has_depth_(false),
      has_pose_(false),
      has_audio_(false),
      has_disparity_(false),
      rgb_timebase_(0),
      depth_timebase_(0),
      is_rgb_distorted_(false),
      rgb_distortion_model_(""),
      rgb_distortion_params_left_(""),
      rgb_distortion_params_right_(""),
      depth_distortion_model_(""),
      start_timestamp_(0),
      duration_(0),
      rgb_fps_(0),
      rgb_width_(0),
      rgb_height_(0),
      depth_fps_(0),
      depth_width_(0),
      depth_height_(0),
      keyframe_rgb_idx_(0),
      allframe_rgb_idx_(0),
      keyframe_depth_idx_(0) {
  
  SetFFmpegLogLevel(log_level_);

  if (filename_.find(' ') != std::string::npos) {
    throw std::runtime_error("Find blank in filename, please fix it.");
  }
  if (!fs::exists(filename_)) {
    throw std::runtime_error("Could not open file " + filename_);
  }

  int ret = avformat_open_input(&pFormatCtx_, filename_.c_str(), NULL, NULL);
  if (ret < 0) {
    throw std::runtime_error("Could not open file " + filename_);
  }
  // get stream info
  avformat_find_stream_info(pFormatCtx_, NULL);
  av_dump_format(pFormatCtx_, 0, filename_.c_str(), 0);

  auto stream_type_info = GetStreamTypeInfo(pFormatCtx_);
  for (auto& pair : stream_type_info) {
    int stream_index = pair.first;
    StreamType stream_type = pair.second;
    switch (stream_type) {
      case MEDIA_TYPE_AUDIO:
        has_audio_ = true;
        audio_frame_id_ = stream_index;
        break;
      case MEDIA_TYPE_AUDIO_2:
        has_audio_ = true;
        audio_2_frame_id_ = stream_index;
        break;
      case MEDIA_TYPE_RGB:
        has_rgb_ = true;
        rgb_frame_id_ = stream_index;
        break;
      case MEDIA_TYPE_DISPARITY:
        has_disparity_ = true;
        disparity_frame_id_ = stream_index;
        break;
      case MEDIA_TYPE_POSE:
        has_pose_ = true;
        pose_frame_id_ = stream_index;
        break;
      case MEDIA_TYPE_DEPTH:
        has_depth_ = true;
        depth_frame_id_ = stream_index;
        break;
      default:
        break;
    }
  }
  // get start timestamp
  start_timestamp_ = pFormatCtx_->streams[0]->start_time;
  duration_ = pFormatCtx_->streams[0]->duration;

  if (has_rgb_) {
    rgb_fps_ =
        pFormatCtx_->streams[rgb_frame_id_]->r_frame_rate.num / pFormatCtx_->streams[rgb_frame_id_]->r_frame_rate.den;
    rgb_width_ = pFormatCtx_->streams[rgb_frame_id_]->codecpar->width / 2;  // side-by-side stereo
    rgb_height_ = pFormatCtx_->streams[rgb_frame_id_]->codecpar->height;

    AVDictionaryEntry* tag = NULL;
    while ((tag = av_dict_get(pFormatCtx_->streams[rgb_frame_id_]->metadata, "", tag, AV_DICT_IGNORE_SUFFIX))) {
      if (std::string(tag->key).find("icam_0") != std::string::npos) {
        std::vector<double> intrinsics_data = SpatialML::String2DoubleVector(tag->value);
        rgb_intrinsics_left_.fx = intrinsics_data[0];
        rgb_intrinsics_left_.fy = intrinsics_data[1];
        rgb_intrinsics_left_.cx = intrinsics_data[2];
        rgb_intrinsics_left_.cy = intrinsics_data[3];
      } else if (std::string(tag->key).find("icam_1") != std::string::npos) {
        std::vector<double> intrinsics_data = SpatialML::String2DoubleVector(tag->value);
        rgb_intrinsics_right_.fx = intrinsics_data[0];
        rgb_intrinsics_right_.fy = intrinsics_data[1];
        rgb_intrinsics_right_.cx = intrinsics_data[2];
        rgb_intrinsics_right_.cy = intrinsics_data[3];
      } else if (std::string(tag->key).find("ecam_0") != std::string::npos) {
        std::vector<double> extrinsics_data = SpatialML::String2DoubleVector(tag->value);
        rgb_extrinsics_left_.extrinsics =
            cv::Matx44d(extrinsics_data[0], extrinsics_data[1], extrinsics_data[2], extrinsics_data[3],
                        extrinsics_data[4], extrinsics_data[5], extrinsics_data[6], extrinsics_data[7],
                        extrinsics_data[8], extrinsics_data[9], extrinsics_data[10], extrinsics_data[11], 0, 0, 0, 1);
      } else if (std::string(tag->key).find("ecam_1") != std::string::npos) {
        std::vector<double> extrinsics_data = SpatialML::String2DoubleVector(tag->value);
        rgb_extrinsics_right_.extrinsics =
            cv::Matx44d(extrinsics_data[0], extrinsics_data[1], extrinsics_data[2], extrinsics_data[3],
                        extrinsics_data[4], extrinsics_data[5], extrinsics_data[6], extrinsics_data[7],
                        extrinsics_data[8], extrinsics_data[9], extrinsics_data[10], extrinsics_data[11], 0, 0, 0, 1);
      } else if (std::string(tag->key).find("distortion_model") != std::string::npos) {
        is_rgb_distorted_ = true;
        rgb_distortion_model_ = tag->value;
      } else if (std::string(tag->key).find("distortion_params_0") != std::string::npos) {
        rgb_distortion_params_left_ = tag->value;
      } else if (std::string(tag->key).find("distortion_params_1") != std::string::npos) {
        rgb_distortion_params_right_ = tag->value;
      } else if (std::string(tag->key).find("timebase") != std::string::npos) {
        rgb_timebase_ = std::stoul(tag->value);
      }
    }
    AVPacket packet;
    av_seek_frame(pFormatCtx_, rgb_frame_id_, 0, AVSEEK_FLAG_BACKWARD);
    while (av_read_frame(pFormatCtx_, &packet) >= 0) {
      if (packet.stream_index == rgb_frame_id_) {
        allframes_rgb.push_back(packet.pts);
        if (packet.flags & AV_PKT_FLAG_KEY) {
          keyframes_rgb.push_back(packet.pts);
        }
      }
      av_packet_unref(&packet);
    }
    if (allframes_rgb.size() != pFormatCtx_->streams[rgb_frame_id_]->nb_frames) {
      throw std::runtime_error("allframes_rgb size(" + std::to_string(allframes_rgb.size()) +
                               ") is not equal to rgb frame number(" +
                               std::to_string(pFormatCtx_->streams[rgb_frame_id_]->nb_frames) + ")");
    }
    spdlog::info("keyframes_rgb size: {}", keyframes_rgb.size());
    spdlog::info("allframes_rgb size: {}", allframes_rgb.size());
    av_seek_frame(pFormatCtx_, -1, 0, AVSEEK_FLAG_BACKWARD);

    // init rgb codec and codec context
    rgb_codec_ = avcodec_find_decoder(pFormatCtx_->streams[rgb_frame_id_]->codecpar->codec_id);
    if (!rgb_codec_) {
      throw std::runtime_error("RGB codec not found");
    }
    rgb_codec_ctx_ = avcodec_alloc_context3(rgb_codec_);
    if (!rgb_codec_ctx_) {
      throw std::runtime_error("Could not allocate codec context");
    }
    if (avcodec_parameters_to_context(rgb_codec_ctx_, pFormatCtx_->streams[rgb_frame_id_]->codecpar) < 0) {
      throw std::runtime_error("Could not copy codec parameters");
    }
    if (avcodec_open2(rgb_codec_ctx_, rgb_codec_, NULL) < 0) {
      throw std::runtime_error("Could not open codec");
    }
    rgb_frame_ = av_frame_alloc();
  }

  if (has_depth_) {
    depth_fps_ = pFormatCtx_->streams[depth_frame_id_]->r_frame_rate.num /
                 pFormatCtx_->streams[depth_frame_id_]->r_frame_rate.den;
    depth_width_ = pFormatCtx_->streams[depth_frame_id_]->codecpar->width;
    depth_height_ = pFormatCtx_->streams[depth_frame_id_]->codecpar->height;

    AVDictionaryEntry* tag = NULL;
    while ((tag = av_dict_get(pFormatCtx_->streams[depth_frame_id_]->metadata, "", tag, AV_DICT_IGNORE_SUFFIX))) {
      if (std::string(tag->key).find("icam_0") != std::string::npos) {
        std::vector<double> intrinsics_data = SpatialML::String2DoubleVector(tag->value);
        depth_intrinsics_.fx = intrinsics_data[0];
        depth_intrinsics_.fy = intrinsics_data[1];
        depth_intrinsics_.cx = intrinsics_data[2];
        depth_intrinsics_.cy = intrinsics_data[3];
      } else if (std::string(tag->key).find("ecam_0") != std::string::npos) {
        std::vector<double> extrinsics_data = SpatialML::String2DoubleVector(tag->value);
        depth_extrinsics_.extrinsics =
            cv::Matx44d(extrinsics_data[0], extrinsics_data[1], extrinsics_data[2], extrinsics_data[3],
                        extrinsics_data[4], extrinsics_data[5], extrinsics_data[6], extrinsics_data[7],
                        extrinsics_data[8], extrinsics_data[9], extrinsics_data[10], extrinsics_data[11], 0, 0, 0, 1);
      } else if (std::string(tag->key).find("distortion_model") != std::string::npos) {
        depth_distortion_model_ = tag->value;
      } else if (std::string(tag->key).find("distortion_params_0") != std::string::npos) {
        depth_distortion_params_ = tag->value;
      } else if (std::string(tag->key).find("timebase") != std::string::npos) {
        depth_timebase_ = std::stoul(tag->value);
      }
    }
    AVPacket packet;
    av_seek_frame(pFormatCtx_, depth_frame_id_, 0, AVSEEK_FLAG_BACKWARD);
    while (av_read_frame(pFormatCtx_, &packet) >= 0) {
      if (packet.stream_index == depth_frame_id_ && (packet.flags & AV_PKT_FLAG_KEY)) {
        keyframes_depth.push_back(packet.pts);
      }
      av_packet_unref(&packet);
    }
    if (keyframes_depth.size() != pFormatCtx_->streams[depth_frame_id_]->nb_frames) {
      throw std::runtime_error("keyframes_depth size(" + std::to_string(keyframes_depth.size()) +
                               ") is not equal to depth frame number(" +
                               std::to_string(pFormatCtx_->streams[depth_frame_id_]->nb_frames) + ")");
    }
    spdlog::info("keyframes_depth size: {}", keyframes_depth.size());
    av_seek_frame(pFormatCtx_, -1, 0, AVSEEK_FLAG_BACKWARD);
  }

  if (has_pose_) {
    LoadAllPoseData(pose_frame_id_);
    spdlog::info("pose_frames size: {}", pose_frames_.size());
  }
}

Reader::~Reader() {
  if (pFormatCtx_ != NULL) {
    avformat_close_input(&pFormatCtx_);
  }
  if (has_pose_ && pose_frames_.size() > 0) {
    pose_frames_.clear();
  }
  if (has_rgb_) {
    avcodec_free_context(&rgb_codec_ctx_);
    av_frame_free(&rgb_frame_);
  }
}

bool Reader::HasNext() const {
  if (pFormatCtx_ == NULL) {
    std::cerr << "pFormatCtx_ is NULL" << std::endl;
    return false;
  }
  switch (read_mode_) {
    case ReadMode::DEPTH_ONLY:
    case ReadMode::DEPTH_FIRST:
      if (keyframes_depth.size() == 0) {
        std::cerr << "keyframes_depth is empty" << std::endl;
        return false;
      }
      return keyframe_depth_idx_ < keyframes_depth.size();
    case ReadMode::RGB_ONLY:
      if (allframes_rgb.size() == 0) {
        std::cerr << "allframes_rgb is empty" << std::endl;
        return false;
      }
      return allframe_rgb_idx_ < allframes_rgb.size();
    default:
      return false;
  }
}

void Reader::Reset() {
  if (pFormatCtx_ == NULL) {
    std::cerr << "pFormatCtx_ is NULL" << std::endl;
    return;
  }
  av_seek_frame(pFormatCtx_, -1, 0, AVSEEK_FLAG_BACKWARD);
  keyframe_rgb_idx_ = 0;
  allframe_rgb_idx_ = 0;
  keyframe_depth_idx_ = 0;
}

int Reader::GetIndex() const {
  switch (read_mode_) {
    case ReadMode::DEPTH_FIRST:
    case ReadMode::DEPTH_ONLY:
      return keyframe_depth_idx_ - 1;
    case ReadMode::RGB_ONLY:
      return allframe_rgb_idx_ - 1;
    default:
      return -1;
  }
}

int Reader::GetFrameCount() const {
  switch (read_mode_) {
    case ReadMode::DEPTH_FIRST:
    case ReadMode::DEPTH_ONLY:
      return keyframes_depth.size();
    case ReadMode::RGB_ONLY:
      return allframes_rgb.size();
    default:
      return -1;
  }
}

void Reader::Load(rgb_frame& frame_rgb, depth_frame& frame_depth) {
  if (pFormatCtx_ == NULL) {
    std::cerr << "pFormatCtx_ is NULL" << std::endl;
    return;
  }
  if (read_mode_ != ReadMode::DEPTH_FIRST) {
    throw std::runtime_error("Read mode should be DEPTH_FIRST, but got " + std::to_string(read_mode_));
  }
  if (!has_depth_ || !has_rgb_) {
    std::cerr << "depth frame or rgb frame is not found" << std::endl;
    return;
  }
  if (rgb_frame_pts_queue_.size() == 0) {
    for (auto rgb_frame_pts : allframes_rgb) {
      double rgb_timestamp = rgb_frame_pts * av_q2d(pFormatCtx_->streams[rgb_frame_id_]->time_base);
      rgb_frame_pts_queue_.addPose(rgb_timestamp, rgb_frame_pts);
    }
  }
  // rgb_frame_pts_queue_.print();

  av_seek_frame(pFormatCtx_, depth_frame_id_, keyframes_depth[keyframe_depth_idx_], AVSEEK_FLAG_BACKWARD);
  while (av_read_frame(pFormatCtx_, &current_packet_) >= 0) {
    if (current_packet_.stream_index == depth_frame_id_) {
      if (current_packet_.pts != keyframes_depth[keyframe_depth_idx_]) {
        av_packet_unref(&current_packet_);
        continue;
      }
      ParseDepthFrame(current_packet_, frame_depth, true);
      keyframe_depth_idx_++;
      break;
    }
    av_packet_unref(&current_packet_);
  }
  // frame_depth.timestamp = keyframes_depth[keyframe_depth_idx_] *
  // av_q2d(pFormatCtx_->streams[depth_frame_id_]->time_base); keyframe_depth_idx_++;

  // 1. find the nearest rgb frame from rgb_frame_pts_queue_
  int64_t target_rgb_pts;
  double time_diff;
  double depth_timestamp = frame_depth.timestamp;
  if (!rgb_frame_pts_queue_.findNearestPose(depth_timestamp, target_rgb_pts, time_diff)) {
    std::cerr << "findNearestPose failed" << std::endl;
    return;
  }
  if (time_diff > 0.1) {
    spdlog::warn("time_diff is too large: {:.4f}", time_diff);
  }

  // 2. get nearest rgb packet
  auto compare = [](int64_t a, int64_t b) { return a < b; };
  auto it = std::upper_bound(keyframes_rgb.begin(), keyframes_rgb.end(), target_rgb_pts, compare);
  if (it == keyframes_rgb.end()) {
    it = keyframes_rgb.end() - 1;
  } else if (it != keyframes_rgb.begin()) {
    --it;
  }
  // seek to the nearest rgb keyframe
  SeekToRgbKeyframe(*it);

  AVPacket pkt;
  AVFrame* frame = av_frame_alloc();
  bool is_first_run = true;
  int64_t packet_pts = -1;
  while (is_first_run || packet_pts <= target_rgb_pts) {
    is_first_run = false;
    if (avio_feof(pFormatCtx_->pb)) {
      spdlog::error("avio_feof");
      break;
    }
    int ret = av_read_frame(pFormatCtx_, &pkt);
    if (ret < 0) {
      spdlog::warn("Error av_read_frame: {}", SpatialML::FFmpegErrorString(ret));
      av_packet_unref(&pkt);
      break;
    }
    if (pkt.stream_index != rgb_frame_id_) {
      av_packet_unref(&pkt);
      continue;
    }
    packet_pts = pkt.pts;

    bool skip = packet_pts != target_rgb_pts;
    ParseRgbFrame(pkt, frame_rgb, skip);
    if (!skip) {
      spdlog::debug("decode target_pts: {}, packet.pts: {}, rgb_timestamp: {:.4f}", target_rgb_pts, pkt.pts,
                    frame_rgb.timestamp);
      break;
    }
    av_packet_unref(&pkt);
  }
  av_frame_free(&frame);
}

void Reader::Load(rgb_frame& frame_rgb) {
  if (pFormatCtx_ == NULL) {
    std::cerr << "pFormatCtx_ is NULL" << std::endl;
    return;
  }
  if (!has_rgb_) {
    std::cerr << "rgb frame is not found" << std::endl;
    return;
  }
  if (read_mode_ != ReadMode::RGB_ONLY) {
    std::cerr << "Read mode should be RGB_ONLY, but got " << ReadMode2String(read_mode_) << std::endl;
    return;
  }
  while (av_read_frame(pFormatCtx_, &current_packet_) >= 0) {
    if (current_packet_.stream_index == rgb_frame_id_) {
      ParseRgbFrame(current_packet_, frame_rgb);
      allframe_rgb_idx_++;
      break;
    }
    av_packet_unref(&current_packet_);
  }
}

void Reader::Load(depth_frame& frame_depth, bool raw_head_pose) {
  if (pFormatCtx_ == NULL) {
    std::cerr << "pFormatCtx_ is NULL" << std::endl;
    return;
  }
  if (!has_depth_) {
    std::cerr << "depth frame is not found" << std::endl;
    return;
  }
  if (read_mode_ != ReadMode::DEPTH_ONLY) {
    std::cerr << "Read mode should be DEPTH_ONLY, but got " << ReadMode2String(read_mode_) << std::endl;
    return;
  }
  while (av_read_frame(pFormatCtx_, &current_packet_) >= 0) {
    if (current_packet_.stream_index == depth_frame_id_) {
      ParseDepthFrame(current_packet_, frame_depth, raw_head_pose);
      keyframe_depth_idx_++;
      break;
    }
    av_packet_unref(&current_packet_);
  }
}

void Reader::Load(Utilities::Rgbd& rgbd, bool densify) {
  if (read_mode_ != ReadMode::DEPTH_FIRST) {
    std::cerr << "Read mode should be DEPTH_FIRST, but got " << ReadMode2String(read_mode_) << std::endl;
    return;
  }
  if (!has_depth_ || !has_rgb_) {
    std::cerr << "depth frame or rgb frame is not found" << std::endl;
    return;
  }

  rgb_frame frame_rgb;
  depth_frame frame_depth;
  Load(frame_rgb, frame_depth);

  // project depth to rgb
  cv::Mat projected_depth;
  auto T_I_Srgb = GetRgbExtrinsicsLeft().as_se3();
  auto T_I_Stof = GetDepthExtrinsics().as_se3();

  // Read head_model_offset from /system/etc/pvr/config/config_head.txt#line_1
  auto head_model_offset = Eigen::Vector3d(-0.05057, -0.01874, 0.04309);

  // Note:
  //   W: World
  //   H: Head
  //   S: Sensor, rgb sensor or depth sensor
  //   I: IMU
  auto T_W_Hrgb = frame_rgb.pose.as_se3();
  auto T_W_Htof = frame_depth.pose.as_se3();
  Sophus::SE3d T_W_Irgb, T_W_Itof;
  Utilities::HeadToImu(T_W_Hrgb, head_model_offset, T_W_Irgb);
  Utilities::HeadToImu(T_W_Htof, head_model_offset, T_W_Itof);
  auto T_W_Srgb = T_W_Irgb * T_I_Srgb;
  auto T_W_Stof = T_W_Itof * T_I_Stof;
  auto T_Srgb_Stof = T_W_Srgb.inverse() * T_W_Stof;

  cv::Matx33d K_tof = GetDepthIntrinsics().as_cvmat();
  if (densify) {
    // densify depth by nearest neighbor interpolation
    float scale = frame_rgb.left_rgb.cols / frame_depth.depth.cols;
    cv::resize(frame_depth.depth, frame_depth.depth, cv::Size(frame_rgb.left_rgb.cols, frame_rgb.left_rgb.rows),
               cv::INTER_NEAREST);
    K_tof = GetDepthIntrinsics().as_cvmat();
    K_tof(0, 0) *= scale;
    K_tof(1, 1) *= scale;
    K_tof(0, 2) *= scale;
    K_tof(1, 2) *= scale;
  }

  Utilities::ProjectDepthToRgb(frame_depth.depth, frame_rgb.left_rgb, GetRgbIntrinsicsLeft().as_cvmat(), K_tof,
                               T_Srgb_Stof, projected_depth, true);
  // timestamp is the time of depth frame
  // T_W_Srgb is the transform from world to rgb sensor
  rgbd = Utilities::Rgbd(frame_rgb.left_rgb, projected_depth, frame_depth.timestamp, T_W_Srgb);
  // Why it is bad with T_W_Srgb?
  // rgbd = Utilities::Rgbd(frame_rgb.left_rgb, projected_depth, frame_depth.timestamp, T_W_Stof);
}

void Reader::LoadAllPoseData(int frame_id) {
  const AVStream* pose_stream = pFormatCtx_->streams[frame_id];
  AVPacket pkt;
  while (av_read_frame(pFormatCtx_, &pkt) >= 0) {
    if (pkt.stream_index == frame_id) {
      SpatialML::pose_frame pose_frame;
      pose_frame.timestamp = pkt.pts * av_q2d(pose_stream->time_base);  // in seconds;
      double pose_data[7];
      std::memcpy(pose_data, pkt.data, 7 * sizeof(double));
      pose_frame.x = pose_data[0];
      pose_frame.y = pose_data[1];
      pose_frame.z = pose_data[2];
      pose_frame.qw = pose_data[3];
      pose_frame.qx = pose_data[4];
      pose_frame.qy = pose_data[5];
      pose_frame.qz = pose_data[6];
      pose_frames_.addPose(pose_frame.timestamp, pose_frame);
    }
    av_packet_unref(&pkt);
  }
  av_seek_frame(pFormatCtx_, -1, 0, AVSEEK_FLAG_BACKWARD);
}

void Reader::ParseDepthFrame(const AVPacket& pkt, depth_frame& frame_depth, bool raw_head_pose) {
  frame_depth.timestamp = pkt.pts * av_q2d(pFormatCtx_->streams[depth_frame_id_]->time_base);
  frame_depth.depth = cv::Mat(depth_height_, depth_width_, CV_16UC1, pkt.data);
  frame_depth.depth.convertTo(frame_depth.depth, CV_32FC1, 0.001);  // unit: meter
  pose_frame target_pose;
  double time_diff;
  pose_frames_.findNearestPose(frame_depth.timestamp, target_pose, time_diff);
  frame_depth.pose = target_pose;

  bool pose_is_valid = false;
  if (time_diff <= find_pose_distance_ || IsLastFrame()) {
    frame_depth.pose = target_pose;
    pose_is_valid = true;
  } else {
    frame_depth.pose = pose_frame();
    pose_is_valid = false;
  }
  if (!raw_head_pose && pose_is_valid) {
    // Read head_model_offset from /system/etc/pvr/config/config_head.txt#line_1
    auto head_model_offset = Eigen::Vector3d(-0.05057, -0.01874, 0.04309);
    // convert T_W_Htof to T_W_Stof
    // Note:
    //   W: World
    //   H: Head
    //   S: Sensor, rgb sensor or depth sensor
    //   I: IMU
    auto T_W_Htof = frame_depth.pose.as_se3();
    auto T_I_Stof = GetDepthExtrinsics().as_se3();

    Sophus::SE3d T_W_Itof;
    Utilities::HeadToImu(T_W_Htof, head_model_offset, T_W_Itof);
    auto T_W_Stof = T_W_Itof * T_I_Stof;

    Eigen::Vector3d t = T_W_Stof.translation();
    Eigen::Quaterniond q = T_W_Stof.unit_quaternion();
    frame_depth.pose.x = t.x();
    frame_depth.pose.y = t.y();
    frame_depth.pose.z = t.z();
    frame_depth.pose.qx = q.x();
    frame_depth.pose.qy = q.y();
    frame_depth.pose.qz = q.z();
    frame_depth.pose.qw = q.w();
  }
}

void Reader::ParseRgbFrame(const AVPacket& pkt, rgb_frame& frame_rgb, bool skip) {
  frame_rgb.timestamp = pkt.pts * av_q2d(pFormatCtx_->streams[rgb_frame_id_]->time_base);

  int ret = avcodec_send_packet(rgb_codec_ctx_, &pkt);
  if (ret < 0) {
    throw std::runtime_error("Error sending packet to decoder");
  }
  ret = avcodec_receive_frame(rgb_codec_ctx_, rgb_frame_);
  if (ret < 0) {
    throw std::runtime_error("Error receiving frame from decoder");
  }
  if (!skip) {
    std::pair<cv::Mat, cv::Mat> rgb_pair;
    if (!FrameToBGR24(rgb_frame_, rgb_pair)) {
      throw std::runtime_error("Failed to convert frame to BGR24");
    }
    frame_rgb.left_rgb = rgb_pair.first;
    frame_rgb.right_rgb = rgb_pair.second;
    pose_frame target_pose;
    double time_diff;
    pose_frames_.findNearestPose(frame_rgb.timestamp, target_pose, time_diff);
    if (time_diff <= find_pose_distance_ || IsLastFrame()) {
      frame_rgb.pose = target_pose;
    } else {
      frame_rgb.pose = pose_frame();
    }
  }
}

bool Reader::SeekToRgbKeyframe(int64_t target_pts) {
  // Don't cache history keyframe, since depth frame always do seek.
  int ret = av_seek_frame(pFormatCtx_, rgb_frame_id_, target_pts, AVSEEK_FLAG_BACKWARD | AVSEEK_FLAG_FRAME);
  if (ret < 0) {
    spdlog::error("seek to keyframe failed: {}", SpatialML::FFmpegErrorString(ret));
    return false;
  }
  avcodec_flush_buffers(rgb_codec_ctx_);
  avcodec_parameters_to_context(rgb_codec_ctx_, pFormatCtx_->streams[rgb_frame_id_]->codecpar);
  avcodec_open2(rgb_codec_ctx_, rgb_codec_, NULL);

  spdlog::debug("seek to keyframe_pts: {}", target_pts);
  return true;
}

bool Reader::IsLastFrame() {
  // std::cout << "GetIndex: " << GetIndex() << ", GetFrameCount: " << GetFrameCount() << std::endl;
  return GetIndex() == GetFrameCount() - 1;
}

}  // namespace SpatialML
