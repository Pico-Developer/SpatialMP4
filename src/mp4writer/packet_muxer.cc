#include "mp4writer/packet_muxer.h"

#include <algorithm>
#include <cerrno>
#include <cctype>
#include <climits>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <regex>
#include <stdexcept>

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavutil/dict.h>
#include <libavutil/error.h>
#include <libavutil/frame.h>
#include <libavutil/imgutils.h>
#include <libavutil/mathematics.h>
#include <libavutil/mem.h>
#include <libavutil/opt.h>
}

namespace fs = std::filesystem;

namespace SpatialMP4 {
namespace {

constexpr AVRational kUsTimeBase{1, 1000000};
constexpr int64_t kDefaultFrameDurationUs = 33333;
constexpr AVPacketSideDataType kPktDataEcam = AV_PKT_DATA_ECAM;
constexpr AVPacketSideDataType kPktDataIcam = AV_PKT_DATA_ICAM;
constexpr AVPacketSideDataType kPktDataDistortion = AV_PKT_DATA_DISTORTION_COEFFICIENTS;

struct PlaneLayout {
  int index = 0;
  int row_stride = 0;
  int pixel_stride = 0;
  int64_t offset = 0;
  int64_t size = 0;
};

struct YuvFrameInput {
  fs::path path;
  int width = 0;
  int height = 0;
  std::vector<PlaneLayout> planes;
};

struct RgbPacketInput {
  int64_t pts_us = 0;
  int64_t timestamp_ns = 0;
  YuvFrameInput left;
  YuvFrameInput right;
};

struct DepthPacketInput {
  int64_t pts_us = 0;
  fs::path path;
  int width = 0;
  int height = 0;
};

struct PosePacketInput {
  int64_t pts_us = 0;
  int64_t offset = 0;
  int size = 0;
};

struct PendingPacket {
  int stream_index = 0;
  int64_t pts_us = 0;
  int64_t duration_us = kDefaultFrameDurationUs;
  fs::path path;
  int64_t offset = 0;
  int size = 0;
  size_t input_index = 0;
  enum class Kind { kRgb, kDepth, kPose } kind = Kind::kDepth;
};

std::string ReadText(const fs::path& path) {
  std::ifstream input(path);
  if (!input) {
    throw std::runtime_error("failed to open " + path.string());
  }
  return std::string(std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>());
}

std::vector<uint8_t> ReadBytes(const fs::path& path) {
  std::ifstream input(path, std::ios::binary);
  if (!input) {
    throw std::runtime_error("failed to open " + path.string());
  }
  return std::vector<uint8_t>(std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>());
}

std::vector<uint8_t> ReadBytesAt(const fs::path& path, int64_t offset, int size) {
  std::ifstream input(path, std::ios::binary);
  if (!input) {
    throw std::runtime_error("failed to open " + path.string());
  }
  input.seekg(offset, std::ios::beg);
  std::vector<uint8_t> data(size);
  input.read(reinterpret_cast<char*>(data.data()), size);
  if (input.gcount() != size) {
    throw std::runtime_error("short read from " + path.string());
  }
  return data;
}

std::string FFmpegError(int errnum) {
  char buffer[AV_ERROR_MAX_STRING_SIZE] = {0};
  av_strerror(errnum, buffer, sizeof(buffer));
  return std::string(buffer);
}

std::string ExtractString(const std::string& text, const std::string& key, const std::string& fallback = "") {
  const std::regex pattern("\"" + key + "\"\\s*:\\s*\"([^\"]*)\"");
  std::smatch match;
  if (std::regex_search(text, match, pattern)) {
    return match[1].str();
  }
  return fallback;
}

int64_t ExtractInt64(const std::string& text, const std::string& key, int64_t fallback = 0) {
  const std::regex pattern("\"" + key + "\"\\s*:\\s*(-?[0-9]+)");
  std::smatch match;
  if (std::regex_search(text, match, pattern)) {
    return std::stoll(match[1].str());
  }
  return fallback;
}

std::string ExtractNestedRange(const std::string& text, const std::string& key, char open_char, char close_char) {
  const std::regex pattern("\"" + key + "\"\\s*:\\s*");
  std::smatch match;
  if (!std::regex_search(text, match, pattern)) {
    return "";
  }

  size_t pos = static_cast<size_t>(match.position() + match.length());
  while (pos < text.size() && std::isspace(static_cast<unsigned char>(text[pos]))) {
    ++pos;
  }
  if (pos >= text.size() || text[pos] != open_char) {
    return "";
  }

  int depth = 0;
  bool in_string = false;
  bool escaped = false;
  for (size_t i = pos; i < text.size(); ++i) {
    const char c = text[i];
    if (in_string) {
      if (escaped) {
        escaped = false;
      } else if (c == '\\') {
        escaped = true;
      } else if (c == '"') {
        in_string = false;
      }
      continue;
    }

    if (c == '"') {
      in_string = true;
    } else if (c == open_char) {
      ++depth;
    } else if (c == close_char) {
      --depth;
      if (depth == 0) {
        return text.substr(pos, i - pos + 1);
      }
    }
  }

  return "";
}

std::string ExtractObject(const std::string& text, const std::string& key) {
  return ExtractNestedRange(text, key, '{', '}');
}

std::string ExtractArray(const std::string& text, const std::string& key) {
  return ExtractNestedRange(text, key, '[', ']');
}

std::vector<std::string> SplitTopLevelObjects(const std::string& array_text) {
  std::vector<std::string> objects;
  size_t pos = 0;
  while (pos < array_text.size()) {
    while (pos < array_text.size() && array_text[pos] != '{') {
      ++pos;
    }
    if (pos >= array_text.size()) {
      break;
    }

    int depth = 0;
    bool in_string = false;
    bool escaped = false;
    const size_t start = pos;
    for (; pos < array_text.size(); ++pos) {
      const char c = array_text[pos];
      if (in_string) {
        if (escaped) {
          escaped = false;
        } else if (c == '\\') {
          escaped = true;
        } else if (c == '"') {
          in_string = false;
        }
        continue;
      }

      if (c == '"') {
        in_string = true;
      } else if (c == '{') {
        ++depth;
      } else if (c == '}') {
        --depth;
        if (depth == 0) {
          objects.push_back(array_text.substr(start, pos - start + 1));
          ++pos;
          break;
        }
      }
    }
  }
  return objects;
}

std::vector<std::string> ReadLines(const fs::path& path) {
  std::ifstream input(path);
  if (!input) {
    throw std::runtime_error("failed to open " + path.string());
  }
  std::vector<std::string> lines;
  std::string line;
  while (std::getline(input, line)) {
    if (!line.empty()) {
      lines.push_back(line);
    }
  }
  return lines;
}

std::vector<PlaneLayout> ParsePlaneLayouts(const std::string& metadata_text, const fs::path& frame_path) {
  const std::string planes_text = ExtractArray(metadata_text, "planes");
  const auto plane_objects = SplitTopLevelObjects(planes_text);
  if (plane_objects.size() != 3) {
    throw std::runtime_error("RGB frame metadata must contain 3 YUV planes for " + frame_path.string());
  }

  std::vector<PlaneLayout> planes(3);
  std::vector<bool> seen(3, false);
  for (const auto& object : plane_objects) {
    PlaneLayout plane;
    plane.index = static_cast<int>(ExtractInt64(object, "index", -1));
    plane.row_stride = static_cast<int>(ExtractInt64(object, "row_stride", 0));
    plane.pixel_stride = static_cast<int>(ExtractInt64(object, "pixel_stride", 0));
    plane.offset = ExtractInt64(object, "offset", -1);
    plane.size = ExtractInt64(object, "size", -1);
    if (plane.index < 0 || plane.index >= 3 || seen[plane.index]) {
      throw std::runtime_error("invalid RGB plane index in " + frame_path.string());
    }
    if (plane.row_stride <= 0 || plane.pixel_stride <= 0 || plane.offset < 0 || plane.size < 0) {
      throw std::runtime_error("invalid RGB plane layout in " + frame_path.string());
    }
    seen[plane.index] = true;
    planes[plane.index] = plane;
  }
  return planes;
}

YuvFrameInput ParseYuvFrameInput(const fs::path& packet_dir, const std::string& line, const std::string& side) {
  YuvFrameInput input;
  input.path = packet_dir / ExtractString(line, side);
  const std::string metadata_text = ExtractObject(line, side + "_metadata");
  input.width = static_cast<int>(ExtractInt64(metadata_text, "width"));
  input.height = static_cast<int>(ExtractInt64(metadata_text, "height"));
  input.planes = ParsePlaneLayouts(metadata_text, input.path);

  if (input.width <= 0 || input.height <= 0) {
    throw std::runtime_error("RGB frame metadata must include width/height for " + input.path.string());
  }
  if ((input.width % 2) != 0 || (input.height % 2) != 0) {
    throw std::runtime_error("RGB HEVC muxer requires even YUV420 dimensions for " + input.path.string());
  }
  if (!fs::exists(input.path)) {
    throw std::runtime_error("missing RGB payload " + input.path.string());
  }
  return input;
}

std::vector<RgbPacketInput> LoadRgbInputs(const fs::path& packet_dir) {
  std::vector<RgbPacketInput> result;
  for (const auto& line : ReadLines(packet_dir / "rgb" / "stereo_pairs.jsonl")) {
    RgbPacketInput input;
    input.pts_us = ExtractInt64(line, "pts_us");
    input.timestamp_ns = ExtractInt64(line, "timestamp_ns");
    input.left = ParseYuvFrameInput(packet_dir, line, "left");
    input.right = ParseYuvFrameInput(packet_dir, line, "right");
    if (input.left.width != input.right.width || input.left.height != input.right.height) {
      throw std::runtime_error("left/right RGB dimensions must match");
    }
    result.push_back(input);
  }
  return result;
}

std::vector<DepthPacketInput> LoadDepthInputs(const fs::path& packet_dir) {
  std::vector<DepthPacketInput> result;
  for (const auto& line : ReadLines(packet_dir / "depth" / "depth_frames.jsonl")) {
    DepthPacketInput input;
    input.pts_us = ExtractInt64(line, "pts_us");
    input.path = packet_dir / ExtractString(line, "path");
    input.width = static_cast<int>(ExtractInt64(line, "width"));
    input.height = static_cast<int>(ExtractInt64(line, "height"));
    if (input.width <= 0 || input.height <= 0) {
      throw std::runtime_error("invalid depth dimensions in depth_frames.jsonl");
    }
    result.push_back(input);
  }
  return result;
}

std::vector<PosePacketInput> LoadPoseInputs(const fs::path& packet_dir) {
  std::vector<PosePacketInput> result;
  for (const auto& line : ReadLines(packet_dir / "pose" / "head_pose_packets.jsonl")) {
    PosePacketInput input;
    input.pts_us = ExtractInt64(line, "pts_us");
    input.offset = ExtractInt64(line, "offset");
    input.size = static_cast<int>(ExtractInt64(line, "size"));
    if (input.size != 7 * static_cast<int>(sizeof(double))) {
      throw std::runtime_error("invalid head pose packet size");
    }
    result.push_back(input);
  }
  return result;
}

template <typename PacketInput>
int64_t EstimateDurationUs(const std::vector<PacketInput>& inputs, size_t index) {
  if (inputs.size() > 1) {
    if (index + 1 < inputs.size()) {
      const int64_t duration = inputs[index + 1].pts_us - inputs[index].pts_us;
      if (duration > 0) {
        return duration;
      }
    }
    if (index > 0) {
      const int64_t duration = inputs[index].pts_us - inputs[index - 1].pts_us;
      if (duration > 0) {
        return duration;
      }
    }
  }
  return kDefaultFrameDurationUs;
}

template <typename PacketInput>
AVRational EstimateFrameRate(const std::vector<PacketInput>& inputs) {
  int num = 0;
  int den = 1;
  av_reduce(&num, &den, kUsTimeBase.den, EstimateDurationUs(inputs, 0), INT_MAX);
  return AVRational{num, den};
}

void AddStreamSideData(AVStream* stream, AVPacketSideDataType type, const std::vector<uint8_t>& data) {
  AVPacketSideData* side_data = av_packet_side_data_new(&stream->codecpar->coded_side_data,
                                                        &stream->codecpar->nb_coded_side_data, type, data.size(), 0);
  if (!side_data) {
    throw std::runtime_error("failed to allocate stream side data");
  }
  std::memcpy(side_data->data, data.data(), data.size());
}

void CheckPlaneAccess(const std::vector<uint8_t>& src, const PlaneLayout& plane, int width, int height,
                      const fs::path& path) {
  if (width <= 0 || height <= 0) {
    throw std::runtime_error("invalid YUV plane dimensions for " + path.string());
  }
  const int64_t last = plane.offset + static_cast<int64_t>(height - 1) * plane.row_stride +
                       static_cast<int64_t>(width - 1) * plane.pixel_stride;
  if (last < plane.offset || last >= static_cast<int64_t>(src.size()) ||
      last >= plane.offset + plane.size) {
    throw std::runtime_error("YUV plane metadata exceeds payload bounds for " + path.string());
  }
}

void CopyPlaneIntoFrame(const std::vector<uint8_t>& src, const PlaneLayout& plane, int src_width, int src_height,
                        AVFrame* frame, int dst_plane, int dst_x, const fs::path& path) {
  CheckPlaneAccess(src, plane, src_width, src_height, path);
  uint8_t* dst = frame->data[dst_plane];
  const int dst_linesize = frame->linesize[dst_plane];
  for (int y = 0; y < src_height; ++y) {
    const int64_t row_offset = plane.offset + static_cast<int64_t>(y) * plane.row_stride;
    uint8_t* dst_row = dst + static_cast<int64_t>(y) * dst_linesize + dst_x;
    for (int x = 0; x < src_width; ++x) {
      dst_row[x] = src[row_offset + static_cast<int64_t>(x) * plane.pixel_stride];
    }
  }
}

void CopyEyeIntoStereoFrame(const YuvFrameInput& eye, int dst_x, AVFrame* frame) {
  const auto src = ReadBytes(eye.path);
  CopyPlaneIntoFrame(src, eye.planes[0], eye.width, eye.height, frame, 0, dst_x, eye.path);
  CopyPlaneIntoFrame(src, eye.planes[1], eye.width / 2, eye.height / 2, frame, 1, dst_x / 2, eye.path);
  CopyPlaneIntoFrame(src, eye.planes[2], eye.width / 2, eye.height / 2, frame, 2, dst_x / 2, eye.path);
}

AVFrame* BuildStereoYuvFrame(const RgbPacketInput& input, int64_t pts_us, int64_t duration_us) {
  AVFrame* frame = av_frame_alloc();
  if (!frame) {
    throw std::runtime_error("failed to allocate RGB frame");
  }
  frame->format = AV_PIX_FMT_YUV420P;
  frame->width = input.left.width + input.right.width;
  frame->height = input.left.height;
  frame->pts = pts_us;
  frame->duration = duration_us;

  int ret = av_frame_get_buffer(frame, 32);
  if (ret < 0) {
    av_frame_free(&frame);
    throw std::runtime_error("failed to allocate RGB frame buffer: " + FFmpegError(ret));
  }
  ret = av_frame_make_writable(frame);
  if (ret < 0) {
    av_frame_free(&frame);
    throw std::runtime_error("RGB frame is not writable: " + FFmpegError(ret));
  }

  CopyEyeIntoStereoFrame(input.left, 0, frame);
  CopyEyeIntoStereoFrame(input.right, input.left.width, frame);
  return frame;
}

void ReceiveAndWriteEncodedPackets(AVFormatContext* format_context, AVCodecContext* encoder_context, AVStream* stream,
                                   PacketMuxerResult* result) {
  AVPacket* packet = av_packet_alloc();
  if (!packet) {
    throw std::runtime_error("failed to allocate encoded RGB packet");
  }

  while (true) {
    const int ret = avcodec_receive_packet(encoder_context, packet);
    if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF) {
      break;
    }
    if (ret < 0) {
      av_packet_free(&packet);
      throw std::runtime_error("failed to receive encoded RGB packet: " + FFmpegError(ret));
    }

    av_packet_rescale_ts(packet, encoder_context->time_base, stream->time_base);
    packet->stream_index = stream->index;
    const int write_ret = av_interleaved_write_frame(format_context, packet);
    av_packet_unref(packet);
    if (write_ret < 0) {
      av_packet_free(&packet);
      throw std::runtime_error("failed to write encoded RGB packet: " + FFmpegError(write_ret));
    }
    if (result) {
      result->rgb_packets++;
    }
  }

  av_packet_free(&packet);
}

void EncodeAndWriteRgbFrame(AVFormatContext* format_context, AVCodecContext* encoder_context, AVStream* stream,
                            const RgbPacketInput& input, int64_t duration_us, PacketMuxerResult* result) {
  AVFrame* frame = BuildStereoYuvFrame(input, input.pts_us, duration_us);
  const int ret = avcodec_send_frame(encoder_context, frame);
  av_frame_free(&frame);
  if (ret < 0) {
    throw std::runtime_error("failed to send RGB frame to HEVC encoder: " + FFmpegError(ret));
  }
  ReceiveAndWriteEncodedPackets(format_context, encoder_context, stream, result);
}

void FlushRgbEncoder(AVFormatContext* format_context, AVCodecContext* encoder_context, AVStream* stream,
                     PacketMuxerResult* result) {
  const int ret = avcodec_send_frame(encoder_context, nullptr);
  if (ret < 0 && ret != AVERROR_EOF) {
    throw std::runtime_error("failed to flush HEVC encoder: " + FFmpegError(ret));
  }
  ReceiveAndWriteEncodedPackets(format_context, encoder_context, stream, result);
}

AVStream* AddRgbStream(AVFormatContext* format_context, const fs::path& packet_dir,
                       const std::vector<RgbPacketInput>& rgb_inputs, int64_t track_base_time,
                       AVCodecContext** encoder_context_out) {
  const AVCodec* codec = avcodec_find_encoder_by_name("libx265");
  if (!codec) {
    codec = avcodec_find_encoder_by_name("hevc_videotoolbox");
  }
  if (!codec) {
    codec = avcodec_find_encoder(AV_CODEC_ID_HEVC);
  }
  if (!codec) {
    throw std::runtime_error("HEVC encoder not found");
  }

  AVStream* stream = avformat_new_stream(format_context, nullptr);
  if (!stream) {
    throw std::runtime_error("failed to create RGB stream");
  }

  AVCodecContext* encoder_context = avcodec_alloc_context3(codec);
  if (!encoder_context) {
    throw std::runtime_error("failed to allocate HEVC encoder context");
  }

  encoder_context->codec_id = codec->id;
  encoder_context->codec_type = AVMEDIA_TYPE_VIDEO;
  encoder_context->width = rgb_inputs.front().left.width + rgb_inputs.front().right.width;
  encoder_context->height = rgb_inputs.front().left.height;
  encoder_context->pix_fmt = AV_PIX_FMT_YUV420P;
  encoder_context->time_base = kUsTimeBase;
  encoder_context->framerate = EstimateFrameRate(rgb_inputs);
  encoder_context->sample_aspect_ratio = AVRational{1, 1};
  encoder_context->gop_size = 30;
  encoder_context->max_b_frames = 0;
  encoder_context->bit_rate = std::max<int64_t>(2000000, static_cast<int64_t>(encoder_context->width) *
                                                            encoder_context->height * 4);
  if (format_context->oformat->flags & AVFMT_GLOBALHEADER) {
    encoder_context->flags |= AV_CODEC_FLAG_GLOBAL_HEADER;
  }

  if (encoder_context->priv_data) {
    av_opt_set(encoder_context->priv_data, "preset", "ultrafast", 0);
    av_opt_set(encoder_context->priv_data, "tune", "zerolatency", 0);
    av_opt_set(encoder_context->priv_data, "x265-params", "log-level=error", 0);
  }

  int ret = avcodec_open2(encoder_context, codec, nullptr);
  if (ret < 0) {
    avcodec_free_context(&encoder_context);
    throw std::runtime_error("failed to open HEVC encoder: " + FFmpegError(ret));
  }

  stream->time_base = encoder_context->time_base;
  stream->avg_frame_rate = encoder_context->framerate;
  stream->r_frame_rate = encoder_context->framerate;
  ret = avcodec_parameters_from_context(stream->codecpar, encoder_context);
  if (ret < 0) {
    avcodec_free_context(&encoder_context);
    throw std::runtime_error("failed to copy HEVC encoder parameters: " + FFmpegError(ret));
  }

  av_dict_set(&stream->metadata, "camera_model", "pinhole", 0);
  av_dict_set(&stream->metadata, "distortion_model", "brown", 0);
  av_dict_set(&stream->metadata, "cam_count", "2", 0);
  av_dict_set_int(&stream->metadata, "track_base_time", track_base_time, 0);

  AddStreamSideData(stream, kPktDataIcam, ReadBytes(packet_dir / "muxer" / "rgb_icam.bin"));
  AddStreamSideData(stream, kPktDataEcam, ReadBytes(packet_dir / "muxer" / "rgb_ecam.bin"));
  AddStreamSideData(stream, kPktDataDistortion, ReadBytes(packet_dir / "muxer" / "rgb_dstr.bin"));
  *encoder_context_out = encoder_context;
  return stream;
}

AVStream* AddDepthStream(AVFormatContext* format_context, const fs::path& packet_dir,
                         const std::vector<DepthPacketInput>& depth_inputs, int64_t track_base_time) {
  AVStream* stream = avformat_new_stream(format_context, nullptr);
  if (!stream) {
    throw std::runtime_error("failed to create depth stream");
  }

  stream->time_base = kUsTimeBase;
  stream->avg_frame_rate = EstimateFrameRate(depth_inputs);
  stream->r_frame_rate = stream->avg_frame_rate;

  AVCodecParameters* codecpar = stream->codecpar;
  codecpar->codec_type = AVMEDIA_TYPE_VIDEO;
  codecpar->codec_id = AV_CODEC_ID_NONE;
  codecpar->codec_tag = MKTAG('r', 'a', 'w', '1');
  codecpar->format = AV_PIX_FMT_GRAY16LE;
  codecpar->width = depth_inputs.front().width;
  codecpar->height = depth_inputs.front().height;

  av_dict_set_int(&stream->metadata, "data_accuracy", 2, 0);
  av_dict_set_int(&stream->metadata, "depth_legal_range", 1000, 0);
  av_dict_set(&stream->metadata, "depth_data_precision", "dtmm", 0);
  av_dict_set(&stream->metadata, "camera_model", "pinhole", 0);
  av_dict_set(&stream->metadata, "distortion_model", "brown", 0);
  av_dict_set(&stream->metadata, "cam_count", "1", 0);
  av_dict_set_int(&stream->metadata, "track_base_time", track_base_time, 0);

  AddStreamSideData(stream, kPktDataIcam, ReadBytes(packet_dir / "muxer" / "depth_icam.bin"));
  AddStreamSideData(stream, kPktDataEcam, ReadBytes(packet_dir / "muxer" / "depth_ecam.bin"));
  AddStreamSideData(stream, kPktDataDistortion, ReadBytes(packet_dir / "muxer" / "depth_dstr.bin"));
  return stream;
}

AVStream* AddPoseStream(AVFormatContext* format_context, int64_t track_base_time) {
  AVStream* stream = avformat_new_stream(format_context, nullptr);
  if (!stream) {
    throw std::runtime_error("failed to create pose stream");
  }
  stream->time_base = kUsTimeBase;

  AVCodecParameters* codecpar = stream->codecpar;
  codecpar->codec_type = AVMEDIA_TYPE_DATA;
  codecpar->codec_id = AV_CODEC_ID_NONE;
  codecpar->codec_tag = MKTAG('m', 'e', 't', 't');

  // Identity in handler_name survives the mp4 round-trip; see reader.cc
  // ParseSpatialHandler. The plain metadata below does not always survive.
  av_dict_set(&stream->metadata, "handler_name", "spatialmp4:rigid_pose:head", 0);
  av_dict_set_int(&stream->metadata, "pose_coordinate", 1, 0);
  av_dict_set_int(&stream->metadata, "data_accuracy", 2, 0);
  av_dict_set(&stream->metadata, "pose_position", "head", 0);
  av_dict_set(&stream->metadata, "mime_type", "application/pose", 0);
  av_dict_set_int(&stream->metadata, "track_base_time", track_base_time, 0);
  return stream;
}

void WritePacket(AVFormatContext* format_context, int stream_index, int64_t pts_us, int64_t duration_us,
                 const std::vector<uint8_t>& data) {
  AVPacket* packet = av_packet_alloc();
  if (!packet) {
    throw std::runtime_error("failed to allocate packet");
  }
  int ret = av_new_packet(packet, data.size());
  if (ret < 0) {
    av_packet_free(&packet);
    throw std::runtime_error("failed to allocate packet payload: " + FFmpegError(ret));
  }

  std::memcpy(packet->data, data.data(), data.size());
  packet->stream_index = stream_index;
  packet->pts = pts_us;
  packet->dts = pts_us;
  packet->duration = duration_us;
  packet->flags = AV_PKT_FLAG_KEY;

  ret = av_interleaved_write_frame(format_context, packet);
  av_packet_free(&packet);
  if (ret < 0) {
    throw std::runtime_error("failed to write packet: " + FFmpegError(ret));
  }
}

}  // namespace

PacketMuxerResult PacketMuxer::Mux(const std::string& packet_dir_string, const std::string& output_mp4) {
  const fs::path packet_dir = fs::absolute(packet_dir_string);
  const std::string manifest = ReadText(packet_dir / "package_manifest.json");
  const int64_t track_base_time = ExtractInt64(manifest, "session_start_unix_us");

  auto rgb_inputs = LoadRgbInputs(packet_dir);
  auto depth_inputs = LoadDepthInputs(packet_dir);
  auto pose_inputs = LoadPoseInputs(packet_dir);
  if (rgb_inputs.empty()) {
    throw std::runtime_error("packet package has no RGB frame pairs");
  }
  if (depth_inputs.empty()) {
    throw std::runtime_error("packet package has no depth packets");
  }
  if (pose_inputs.empty()) {
    throw std::runtime_error("packet package has no head pose packets");
  }

  AVFormatContext* format_context = nullptr;
  int ret = avformat_alloc_output_context2(&format_context, nullptr, "mp4", output_mp4.c_str());
  if (ret < 0 || !format_context) {
    throw std::runtime_error("failed to allocate output context: " + FFmpegError(ret));
  }

  PacketMuxerResult result;
  AVCodecContext* rgb_encoder_context = nullptr;
  try {
    AVStream* rgb_stream = AddRgbStream(format_context, packet_dir, rgb_inputs, track_base_time, &rgb_encoder_context);
    AVStream* depth_stream = AddDepthStream(format_context, packet_dir, depth_inputs, track_base_time);
    AVStream* pose_stream = AddPoseStream(format_context, track_base_time);

    ret = avio_open(&format_context->pb, output_mp4.c_str(), AVIO_FLAG_WRITE);
    if (ret < 0) {
      throw std::runtime_error("failed to open output file: " + FFmpegError(ret));
    }

    ret = avformat_write_header(format_context, nullptr);
    if (ret < 0) {
      throw std::runtime_error("failed to write mp4 header: " + FFmpegError(ret));
    }

    std::vector<PendingPacket> pending;
    for (size_t i = 0; i < rgb_inputs.size(); ++i) {
      PendingPacket packet;
      packet.stream_index = rgb_stream->index;
      packet.pts_us = rgb_inputs[i].pts_us;
      packet.duration_us = EstimateDurationUs(rgb_inputs, i);
      packet.input_index = i;
      packet.kind = PendingPacket::Kind::kRgb;
      pending.push_back(packet);
    }
    for (size_t i = 0; i < depth_inputs.size(); ++i) {
      PendingPacket packet;
      packet.stream_index = depth_stream->index;
      packet.pts_us = depth_inputs[i].pts_us;
      packet.duration_us = EstimateDurationUs(depth_inputs, i);
      packet.path = depth_inputs[i].path;
      packet.kind = PendingPacket::Kind::kDepth;
      pending.push_back(packet);
    }
    const fs::path pose_payload = packet_dir / "pose" / "head_pose_mett.bin";
    for (size_t i = 0; i < pose_inputs.size(); ++i) {
      PendingPacket packet;
      packet.stream_index = pose_stream->index;
      packet.pts_us = pose_inputs[i].pts_us;
      packet.duration_us = EstimateDurationUs(pose_inputs, i);
      packet.path = pose_payload;
      packet.offset = pose_inputs[i].offset;
      packet.size = pose_inputs[i].size;
      packet.kind = PendingPacket::Kind::kPose;
      pending.push_back(packet);
    }
    std::sort(pending.begin(), pending.end(), [](const PendingPacket& a, const PendingPacket& b) {
      if (a.pts_us != b.pts_us) return a.pts_us < b.pts_us;
      return a.stream_index < b.stream_index;
    });

    for (const auto& item : pending) {
      if (item.kind == PendingPacket::Kind::kRgb) {
        EncodeAndWriteRgbFrame(format_context, rgb_encoder_context, rgb_stream,
                               rgb_inputs[item.input_index], item.duration_us, &result);
      } else if (item.kind == PendingPacket::Kind::kPose) {
        WritePacket(format_context, item.stream_index, item.pts_us, item.duration_us,
                    ReadBytesAt(item.path, item.offset, item.size));
        result.pose_packets++;
      } else {
        WritePacket(format_context, item.stream_index, item.pts_us, item.duration_us, ReadBytes(item.path));
        result.depth_packets++;
      }
    }
    FlushRgbEncoder(format_context, rgb_encoder_context, rgb_stream, &result);

    ret = av_write_trailer(format_context);
    if (ret < 0) {
      throw std::runtime_error("failed to write mp4 trailer: " + FFmpegError(ret));
    }
  } catch (...) {
    if (format_context->pb) {
      avio_closep(&format_context->pb);
    }
    avcodec_free_context(&rgb_encoder_context);
    avformat_free_context(format_context);
    throw;
  }

  if (format_context->pb) {
    avio_closep(&format_context->pb);
  }
  avcodec_free_context(&rgb_encoder_context);
  avformat_free_context(format_context);
  return result;
}

}  // namespace SpatialMP4
