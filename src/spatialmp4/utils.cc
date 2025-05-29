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

#include "spatialmp4/utils.h"
#include <iostream>
#include <unordered_map>

namespace SpatialML {

std::vector<double> String2DoubleVector(const std::string& str) {
  std::vector<double> result;
  std::stringstream ss(str);
  std::string item;
  while (std::getline(ss, item, ',')) {
    result.push_back(std::stod(item));
  }
  return result;
}

std::string StreamTypeToString(StreamType type) {
  switch (type) {
    case MEDIA_TYPE_AUDIO:
      return "audio";
    case MEDIA_TYPE_RGB:
      return "rgb";
    case MEDIA_TYPE_DISPARITY:
      return "disparity";
    case MEDIA_TYPE_POSE:
      return "pose";
    case MEDIA_TYPE_DEPTH:
      return "depth";
    default:
      return "unknown";
  }
}

void PrintStreamInfo(AVFormatContext* pFormatCtx) {
  const std::unordered_map<int, std::string> STREAM_TYPE_MAP = {{0, "audio"},     {1, "audio"}, {2, "rgb"},
                                                                {3, "disparity"}, {4, "pose"},  {5, "depth"}};

  std::cout << "Stream info:" << std::endl;
  std::cout << "  Number of streams: " << pFormatCtx->nb_streams << std::endl;
  for (int i = 0; i < pFormatCtx->nb_streams; i++) {
    std::cout << "  Stream " << i << ":" << std::endl;
    std::cout << "    Codec type: " << pFormatCtx->streams[i]->codecpar->codec_type
              << "    Codec name: " << avcodec_get_name(pFormatCtx->streams[i]->codecpar->codec_id) << " ("
              << STREAM_TYPE_MAP.at(i) << ")" << std::endl;

    int64_t bit_rate = pFormatCtx->streams[i]->codecpar->bit_rate;
    double kb_rate = bit_rate / 1000.0;
    double mb_rate = bit_rate / 1000000.0;
    std::cout << "    Bitrate: " << bit_rate << " b/s (" << kb_rate << " kb/s, " << mb_rate << " Mb/s)" << std::endl;

    AVDictionaryEntry* tag = NULL;
    std::cout << "    Metadata:" << std::endl;
    while ((tag = av_dict_get(pFormatCtx->streams[i]->metadata, "", tag, AV_DICT_IGNORE_SUFFIX))) {
      std::cout << "      " << tag->key << " = " << tag->value << std::endl;
    }
  }
}

std::unordered_map<int, StreamType> GetStreamTypeInfo(AVFormatContext* pFormatCtx) {
  std::unordered_map<int, StreamType> stream_type_info;
  for (int i = 0; i < pFormatCtx->nb_streams; i++) {
    const char* codec_name = avcodec_get_name(pFormatCtx->streams[i]->codecpar->codec_id);

    if (codec_name == "aac") {
      stream_type_info[i] = MEDIA_TYPE_AUDIO;
    } else if (codec_name == "opus") {
      stream_type_info[i] = MEDIA_TYPE_AUDIO_2;
    } else if (codec_name == "hevc") {
      stream_type_info[i] = MEDIA_TYPE_RGB;
    } else if (codec_name == "disparity") {
      stream_type_info[i] = MEDIA_TYPE_DISPARITY;
    } else if (codec_name == "none") {
      std::vector<std::string> meta_key_list;
      AVDictionaryEntry* tag = NULL;
      while ((tag = av_dict_get(pFormatCtx->streams[i]->metadata, "", tag, AV_DICT_IGNORE_SUFFIX))) {
        meta_key_list.push_back(tag->key);
      }
      if (std::find(meta_key_list.begin(), meta_key_list.end(), "icam_0") != meta_key_list.end()) {
        stream_type_info[i] = MEDIA_TYPE_DEPTH;
      } else {
        stream_type_info[i] = MEDIA_TYPE_POSE;
      }
    } else {
      std::cout << "  Unknown stream type: " << i << ":" << codec_name << std::endl;
    }
  }
  // for (auto& [stream_index, stream_type] : stream_type_info) {
  //   std::cout << "Stream " << stream_index << " is " << StreamTypeToString(stream_type) << std::endl;
  // }
  return stream_type_info;
}

bool FrameToBGR24(AVFrame* rgb_frame, std::pair<cv::Mat, cv::Mat>& rgb_pair) {
  if (rgb_frame == nullptr || rgb_frame->data[0] == nullptr) {
    std::cerr << "rgb_frame is nullptr" << std::endl;
    return false;
  }
  // 1. 检查帧格式
  if (rgb_frame->format != AV_PIX_FMT_RGB24) {
    // 创建转换上下文
    SwsContext* sws_ctx =
        sws_getContext(rgb_frame->width, rgb_frame->height, (AVPixelFormat)rgb_frame->format, rgb_frame->width,
                       rgb_frame->height, AV_PIX_FMT_RGB24, SWS_BILINEAR, nullptr, nullptr, nullptr);
    av_frame_make_writable(rgb_frame);

    if (!sws_ctx) {
      std::cerr << "Failed to create SwsContext" << std::endl;
      return false;
    }

    // 创建目标帧
    AVFrame* rgb24_frame = av_frame_alloc();
    rgb24_frame->format = AV_PIX_FMT_RGB24;
    rgb24_frame->width = rgb_frame->width;
    rgb24_frame->height = rgb_frame->height;

    if (av_frame_get_buffer(rgb24_frame, 0) < 0) {
      std::cerr << "Failed to allocate frame buffer" << std::endl;
      av_frame_free(&rgb24_frame);
      sws_freeContext(sws_ctx);
      return false;
    }
    // 转换格式
    sws_scale(sws_ctx, rgb_frame->data, rgb_frame->linesize, 0, rgb_frame->height, rgb24_frame->data,
              rgb24_frame->linesize);

    // 创建OpenCV Mat
    cv::Mat mat(rgb24_frame->height, rgb24_frame->width, CV_8UC3, rgb24_frame->data[0], rgb24_frame->linesize[0]);
    cv::cvtColor(mat, mat, cv::COLOR_RGB2BGR);

    // 分割左右图像
    cv::Mat left_rgb = mat(cv::Rect(0, 0, mat.cols / 2, mat.rows)).clone();
    cv::Mat right_rgb = mat(cv::Rect(mat.cols / 2, 0, mat.cols / 2, mat.rows)).clone();
    rgb_pair = std::make_pair(left_rgb, right_rgb);

    // 清理资源
    av_frame_free(&rgb24_frame);
    sws_freeContext(sws_ctx);
  } else {
    // 直接使用BGR24格式
    cv::Mat mat(rgb_frame->height, rgb_frame->width, CV_8UC3, rgb_frame->data[0], rgb_frame->linesize[0]);
    cv::cvtColor(mat, mat, cv::COLOR_RGB2BGR);
    cv::Mat left_rgb = mat(cv::Rect(0, 0, mat.cols / 2, mat.rows)).clone();
    cv::Mat right_rgb = mat(cv::Rect(mat.cols / 2, 0, mat.cols / 2, mat.rows)).clone();
    rgb_pair = std::make_pair(left_rgb, right_rgb);
  }
  return true;
}

std::string FFmpegErrorString(int errnum) {
  char buf[AV_ERROR_MAX_STRING_SIZE];
  if (av_strerror(errnum, buf, sizeof(buf)) == 0) {
    return std::string(buf);
  }
  return "unknown error: " + std::to_string(errnum);
}

}  // namespace SpatialML
