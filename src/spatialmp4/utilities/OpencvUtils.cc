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

// OpencvUtils.cc
#include "OpencvUtils.h"

namespace Utilities {

std::string OpencvTypeToString(int type) {
  std::string r;
  uchar depth = type & CV_MAT_DEPTH_MASK;
  uchar chans = 1 + (type >> CV_CN_SHIFT);

  switch (depth) {
    case CV_8U:
      r = "8U";
      break;
    case CV_8S:
      r = "8S";
      break;
    case CV_16U:
      r = "16U";
      break;
    case CV_16S:
      r = "16S";
      break;
    case CV_32S:
      r = "32S";
      break;
    case CV_32F:
      r = "32F";
      break;
    case CV_64F:
      r = "64F";
      break;
    default:
      r = "User";
      break;
  }
  r += "C";
  r += (chans + '0');
  return r;
}

void PrintOpencvMat(const cv::Mat& mat, const std::string& name) {
  std::cout << "Mat " << name << std::endl;
  std::cout << "  size(height, width): " << mat.rows << "x" << mat.cols << std::endl;
  std::cout << "  channels: " << mat.channels() << std::endl;
  std::cout << "  type: " << OpencvTypeToString(mat.type()) << std::endl;
  std::cout << "  depth: " << mat.depth() << std::endl;
  std::cout << "  elemSize: " << mat.elemSize() << std::endl;
  std::cout << "  elemSize1: " << mat.elemSize1() << std::endl;
  std::cout << "  step: " << mat.step << std::endl;
  std::cout << "  data addr: " << static_cast<void*>(mat.data) << std::endl;

  double minVal, maxVal;
  cv::minMaxIdx(mat, &minVal, &maxVal);
  std::cout << "  min: " << minVal << std::endl;
  std::cout << "  max: " << maxVal << std::endl;
  std::cout << "  sum: " << cv::sum(mat) << std::endl;
  std::cout << "  mean: " << cv::mean(mat) << std::endl;
  std::cout << "  data address: " << (void*)mat.data << std::endl;
  std::cout << "  data: " << std::endl;

  for (int i = 0; i < mat.rows; ++i) {
    for (int j = 0; j < mat.cols; ++j) {
      if (CV_MAT_DEPTH(mat.type()) == CV_32F) {
        std::cout << "  " << mat.at<float>(i, j) << " ";
      } else if (CV_MAT_DEPTH(mat.type()) == CV_64F) {
        std::cout << "  " << mat.at<double>(i, j) << " ";
      } else if (CV_MAT_DEPTH(mat.type()) == CV_32S) {
        std::cout << "  " << mat.at<int32_t>(i, j) << " ";
      } else if (CV_MAT_DEPTH(mat.type()) == CV_16S) {
        std::cout << "  " << mat.at<int16_t>(i, j) << " ";
      } else if (CV_MAT_DEPTH(mat.type()) == CV_8S) {
        std::cout << "  " << mat.at<int8_t>(i, j) << " ";
      } else if (CV_MAT_DEPTH(mat.type()) == CV_8U) {
        std::cout << "  " << mat.at<uint8_t>(i, j) << " ";
      } else if (CV_MAT_DEPTH(mat.type()) == CV_16U) {
        std::cout << "  " << mat.at<uint16_t>(i, j) << " ";
      } else {
        std::cout << "  Unknown ";
      }
      if (j > 3) break;
    }
    std::cout << std::endl;
    if (i > 3) break;
  }
}

void VisualizeMat(const cv::Mat depth, cv::Mat& vis_depth, std::string name, cv::Mat* image_ptr, double minVal,
                  double maxVal, bool check_size) {
  double min, max;
  if (minVal == -1 && maxVal == -1) {
    cv::minMaxIdx(depth, &min, &max);
  } else {
    min = minVal;
    max = maxVal;
  }

  std::cout << "Visualize " << name << " min: " << min << ", max: " << max << std::endl;

  depth.convertTo(vis_depth, CV_8UC1, 255.0 / (max - min), -min);
  cv::applyColorMap(vis_depth, vis_depth, cv::COLORMAP_INFERNO);

  if (image_ptr != nullptr) {
    cv::Mat image_resized;
    if (image_ptr->size() != depth.size()) {
      if (check_size) {
        std::cerr << "Error: image size (" << image_ptr->size() << ") is not equal to depth size (" << depth.size()
                  << ")" << std::endl;
        throw std::runtime_error("stop here");
      } else {
        cv::resize(*image_ptr, image_resized, depth.size());
        image_ptr = &image_resized;
      }
    }
    float alpha = 0.4;
    cv::Mat image_float;
    cv::Mat depth_float;
    image_ptr->convertTo(image_float, CV_32FC3);
    vis_depth.convertTo(depth_float, CV_32FC3);
    cv::Mat merged = image_float * alpha + depth_float * (1 - alpha);
    merged.convertTo(vis_depth, CV_8UC3);
  }
}

void DumpMat(const cv::Mat mat, std::string filename) {
  cv::FileStorage fs(filename, cv::FileStorage::WRITE);
  if (!fs.isOpened()) {
    std::cerr << "Error: Could not open the file for writing." << std::endl;
    return;
  }
  fs << "image" << mat;
  fs.release();
}

void LoadMat(const std::string filename, cv::Mat& mat) {
  cv::FileStorage fsRead(filename, cv::FileStorage::READ);
  if (!fsRead.isOpened()) {
    std::cerr << "Error: Could not open the file for reading." << std::endl;
    return;
  }
  fsRead["image"] >> mat;
  fsRead.release();
}

cv::Mat ConcatenateMat(const std::vector<cv::Mat>& mats, const std::vector<std::string>& mats_name,
                       const std::string& save_path) {
  cv::Mat res;
  cv::Size size = mats[0].size();

  for (int i = 0; i < mats.size(); i++) {
    cv::Mat mat_resized;

    if (i == 0) {
      cv::putText(mats[i], mats_name[i], cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 255), 2);
      res = mats[i];
    } else {
      if (mats[i].size().width >= size.width || mats[i].size().height >= size.height) {
        cv::resize(mats[i], mat_resized, size);
        cv::putText(mat_resized, mats_name[i], cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 255),
                    2);
        cv::hconcat(res, mat_resized, res);
      } else {
        cv::Mat mat_bg = cv::Mat::zeros(size, mats[i].type());
        if (mats[i].size().height > size.height) {
          int target_width = size.height / mats[i].size().height * mats[i].size().width;
          cv::resize(mats[i], mat_resized, cv::Size(target_width, size.height));
          cv::Mat roi = mat_bg(cv::Rect((size.width - target_width) / 2, 0, target_width, size.height));
          mat_resized.copyTo(roi);
        } else if (mats[i].size().width > size.width) {
          int target_height = size.width / mats[i].size().width * mats[i].size().height;
          cv::resize(mats[i], mat_resized, cv::Size(size.width, target_height));
          cv::Mat roi = mat_bg(cv::Rect(0, (size.height - target_height) / 2, size.width, target_height));
          mat_resized.copyTo(roi);
        } else {
          cv::Mat roi = mat_bg(
              cv::Rect((size.width - mats[i].cols) / 2, (size.height - mats[i].rows) / 2, mats[i].cols, mats[i].rows));
          mats[i].copyTo(roi);
        }
        cv::putText(mat_bg, mats_name[i], cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 255), 2);
        cv::hconcat(res, mat_bg, res);
      }
    }
  }
  if (!save_path.empty()) {
    cv::imwrite(save_path, res);
  }
  return res;
}

}  // namespace Utilities
