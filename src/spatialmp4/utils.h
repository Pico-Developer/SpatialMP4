#pragma once
#include <iostream>
#include <vector>
#include <string>
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

namespace SpatialML {

std::vector<double> String2DoubleVector(const std::string& str);

std::string StreamTypeToString(StreamType type);

void PrintStreamInfo(AVFormatContext* pFormatCtx);

std::unordered_map<int, StreamType> GetStreamTypeInfo(AVFormatContext* pFormatCtx);

bool FrameToBGR24(AVFrame* rgb_frame, std::pair<cv::Mat, cv::Mat>& rgb_pair);

std::string FFmpegErrorString(int errnum);

}  // namespace SpatialML
