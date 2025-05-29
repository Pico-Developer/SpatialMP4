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

#include <opencv2/opencv.hpp>
#include <sophus/se3.hpp>
#include <Eigen/Eigen>

namespace Utilities {

void ProjectDepthToRgb(const cv::Mat& depth, const cv::Mat& rgb, const cv::Matx33d& K_rgb, const cv::Matx33d& K_depth,
                       const Sophus::SE3d& T_rgb_depth, cv::Mat& projected_depth, bool debug = false);

Sophus::SE3d ReleaseHeadModel(const Sophus::SE3d& T_I, const Eigen::Vector3d& head_model_offset);

Sophus::SE3d ApplyHeadModel(const Sophus::SE3d& T_I, const Eigen::Vector3d& head_model_offset);

void ImuToHead(const Sophus::SE3d& in, const Eigen::Vector3d& head_model_offset, Sophus::SE3d& out);

void HeadToImu(const Sophus::SE3d& in, const Eigen::Vector3d& head_model_offset, Sophus::SE3d& out);

}  // namespace Utilities