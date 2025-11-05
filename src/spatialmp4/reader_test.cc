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

#include <gtest/gtest.h>
#include "spatialmp4/reader.h"
#include "spatialmp4/data_types.h"
#include "spatialmp4/utils.h"
#include "utilities/OpencvUtils.h"
#include "utilities/RgbdUtils.h"
#include "utilities/PointcloudUtils.h"
#include "spdlog/spdlog.h"
#include <sophus/se3.hpp>
#include <fstream>

const std::string kTestFile = "video/test.mp4";
const std::string kVisRgbDir = "./tmp_vis_rgb";
const std::string kVisRgbDir2 = "./tmp_vis_rgb_random";
const std::string kVisDepthDir = "./tmp_vis_depth";
const std::string kVisRgbdDir = "./tmp_vis_rgbd";

std::string GetVideoPath() {
  const char* env = std::getenv("VIDEO");
  if (env) {
    return std::string(env);
  }
  std::ifstream file(kTestFile);
  if (!file.good()) {
    throw std::runtime_error("Please set VIDEO environment variable!");
  }
  file.close();
  return kTestFile;
}

TEST(UtilitiesTest, HeadToImuIdentityOffset) {
  Sophus::SE3d head_pose = Sophus::SE3d();  // Identity pose
  Eigen::Vector3d head_model_offset(0.1, -0.2, 0.3);
  Sophus::SE3d imu_pose;
  Utilities::HeadToImu(head_pose, head_model_offset, imu_pose);

  Eigen::Vector3d expected_translation(-head_model_offset.y(), head_model_offset.x(), -head_model_offset.z());
  EXPECT_TRUE(imu_pose.unit_quaternion().isApprox(head_pose.unit_quaternion(), 1e-12));
  EXPECT_TRUE(imu_pose.translation().isApprox(expected_translation, 1e-12));
}

TEST(UtilitiesTest, HeadToImuIsInverseOfImuToHead) {
  const double pi = 3.14159265358979323846;
  Eigen::Quaterniond q(Eigen::AngleAxisd(pi / 4.0, Eigen::Vector3d::UnitZ()) *
                       Eigen::AngleAxisd(pi / 6.0, Eigen::Vector3d::UnitY()));
  Sophus::SE3d imu_pose(q, Eigen::Vector3d(1.0, -2.0, 0.5));
  Eigen::Vector3d head_model_offset(-0.05, 0.08, -0.12);

  Sophus::SE3d head_pose;
  Utilities::ImuToHead(imu_pose, head_model_offset, head_pose);

  Sophus::SE3d recovered_imu_pose;
  Utilities::HeadToImu(head_pose, head_model_offset, recovered_imu_pose);

  EXPECT_TRUE(recovered_imu_pose.unit_quaternion().isApprox(imu_pose.unit_quaternion(), 1e-12));
  EXPECT_TRUE(recovered_imu_pose.translation().isApprox(imu_pose.translation(), 1e-12));
}

TEST(SpatialMP4Test, FilenameCheck_ReaderTest) {
  EXPECT_THROW(SpatialML::Reader(std::string("video/3DVideo_30min Stationary.mp4")), std::runtime_error);
  EXPECT_THROW(SpatialML::Reader(std::string("video/dont_exists.mp4")), std::runtime_error);
}

TEST(SpatialMP4Test, Basic_ReaderTest) {
  SpatialML::Reader reader(GetVideoPath());
  ASSERT_TRUE(reader.HasRGB());
  ASSERT_TRUE(reader.HasDepth());
  ASSERT_TRUE(reader.HasPose());
  ASSERT_TRUE(reader.HasAudio());
  ASSERT_TRUE(reader.HasDisparity());

  auto rgb_intrinsics_left = reader.GetRgbIntrinsicsLeft();
  auto rgb_intrinsics_right = reader.GetRgbIntrinsicsRight();
  auto rgb_extrinsics_left = reader.GetRgbExtrinsicsLeft();
  auto rgb_extrinsics_right = reader.GetRgbExtrinsicsRight();
  auto depth_intrinsics = reader.GetDepthIntrinsics();
  auto depth_extrinsics = reader.GetDepthExtrinsics();
  std::cout << "rgb_intrinsics_left: " << rgb_intrinsics_left << std::endl;
  std::cout << "rgb_intrinsics_right: " << rgb_intrinsics_right << std::endl;
  std::cout << "rgb_extrinsics_left: " << rgb_extrinsics_left << std::endl;
  std::cout << "rgb_extrinsics_right: " << rgb_extrinsics_right << std::endl;
  std::cout << "depth_intrinsics: " << depth_intrinsics << std::endl;
  std::cout << "depth_extrinsics: " << depth_extrinsics << std::endl;
  std::cout << "is_rgb_distorted: " << (reader.IsRgbDistorted() ? "true" : "false") << std::endl;
  std::cout << "rgb_distortion_model: " << reader.GetRgbDistortionModel() << std::endl;
  std::cout << "rgb_distortion_params_left: " << reader.GetRgbDistortionParamsLeft() << std::endl;
  std::cout << "rgb_distortion_params_right: " << reader.GetRgbDistortionParamsRight() << std::endl;
  std::cout << "depth_distortion_model: " << reader.GetDepthDistortionModel() << std::endl;
  std::cout << "depth_distortion_params: " << reader.GetDepthDistortionParams() << std::endl;
  std::cout << "rgb_timebase: " << SpatialML::microsecondsToDateTime(reader.GetRgbTimebase()) << std::endl;
  std::cout << "depth_timebase: " << SpatialML::microsecondsToDateTime(reader.GetDepthTimebase()) << std::endl;
  std::cout << "start_timestamp: " << reader.GetStartTimestamp() << std::endl;
  std::cout << "duration: " << reader.GetDuration() << std::endl;

  int pose_cnt = 0;
  for (auto pose_frame : reader.GetPoseFrames()) {
    std::cout << pose_frame << std::endl;
    pose_cnt++;
    if (pose_cnt > 10) {
      std::cout << "Timestamp: ..." << std::endl;
      break;
    }
  }

  // get first frame accurate timestamp
  uint64_t rgb_timebase = reader.GetRgbTimebase();
  ASSERT_GT(rgb_timebase, 0);

  uint64_t depth_timebase = reader.GetDepthTimebase();
  ASSERT_GT(depth_timebase, 0);

  float duration = reader.GetDuration();
  ASSERT_GT(duration, 0);

  float rgb_fps = reader.GetRgbFPS();
  ASSERT_GT(rgb_fps, 0);

  int rgb_width = reader.GetRgbWidth();
  ASSERT_GT(rgb_width, 0);

  int rgb_height = reader.GetRgbHeight();
  ASSERT_GT(rgb_height, 0);

  float depth_fps = reader.GetDepthFPS();
  ASSERT_GT(depth_fps, 0);

  int depth_width = reader.GetDepthWidth();
  ASSERT_GT(depth_width, 0);

  int depth_height = reader.GetDepthHeight();
  ASSERT_GT(depth_height, 0);
}

TEST(SpatialMP4Test, DepthFirst_ReaderTest) {
  // spdlog::set_level(spdlog::level::debug); // debug mode

  if (fs::exists(kVisDepthDir)) {
    fs::remove_all(kVisDepthDir);
  }
  fs::create_directory(kVisDepthDir);

  SpatialML::Reader reader(GetVideoPath());
  reader.SetReadMode(SpatialML::Reader::ReadMode::DEPTH_FIRST);
  reader.Reset();
  while (reader.HasNext()) {
    SpatialML::rgb_frame rgb_frame;
    SpatialML::depth_frame depth_frame;
    reader.Load(rgb_frame, depth_frame);

    std::cout << "depth frame: \t" << depth_frame.timestamp << ", rgb frame: \t" << rgb_frame.timestamp << std::endl;
    std::stringstream ss;
    ss << std::setw(4) << std::setfill('0') << reader.GetIndex();

    depth_frame.depth.setTo(0, depth_frame.depth > 10);
    cv::Mat vis_depth;
    Utilities::VisualizeMat(depth_frame.depth, vis_depth, "depth");
    cv::rotate(vis_depth, vis_depth, cv::ROTATE_90_CLOCKWISE);

    // resize depth_frame height alias to rgb_frame.left_rgb height
    float factor = static_cast<float>(rgb_frame.left_rgb.rows) / static_cast<float>(vis_depth.rows);
    cv::resize(vis_depth, vis_depth, cv::Size(vis_depth.cols * factor, vis_depth.rows * factor));
    vis_depth.convertTo(vis_depth, CV_8UC3);

    cv::Mat concat_mat;
    cv::hconcat(rgb_frame.left_rgb.clone(), vis_depth.clone(), concat_mat);

    // project depth to rgb
    cv::Mat projected_depth;
    auto T_I_Srgb = reader.GetRgbExtrinsicsLeft().as_se3();
    auto T_I_Stof = reader.GetDepthExtrinsics().as_se3();

    // Read head_model_offset from /system/etc/pvr/config/config_head.txt#line_1
    auto head_model_offset = Eigen::Vector3d(-0.05057, -0.01874, 0.04309);

    // Note:
    //   W: World
    //   H: Head
    //   S: Sensor, rgb sensor or depth sensor
    //   I: IMU
    auto T_W_Hrgb = rgb_frame.pose.as_se3();
    auto T_W_Htof = depth_frame.pose.as_se3();
    Sophus::SE3d T_W_Irgb, T_W_Itof;
    Utilities::HeadToImu(T_W_Hrgb, head_model_offset, T_W_Irgb);
    Utilities::HeadToImu(T_W_Htof, head_model_offset, T_W_Itof);
    auto T_W_Srgb = T_W_Irgb * T_I_Srgb;
    auto T_W_Stof = T_W_Itof * T_I_Stof;
    auto T_Srgb_Stof = T_W_Srgb.inverse() * T_W_Stof;

    Utilities::ProjectDepthToRgb(depth_frame.depth, rgb_frame.left_rgb, reader.GetRgbIntrinsicsLeft().as_cvmat(),
                                 reader.GetDepthIntrinsics().as_cvmat(), T_Srgb_Stof, projected_depth, true);
    cv::Mat vis_projected_depth;
    Utilities::VisualizeMat(projected_depth, vis_projected_depth, "projected_depth", &rgb_frame.left_rgb, 0, 5, true);
    cv::hconcat(concat_mat, vis_projected_depth, concat_mat);

    std::string filename = kVisDepthDir + "/depth_" + ss.str() + ".png";
    cv::putText(concat_mat, "rgb_" + std::to_string(rgb_frame.timestamp), cv::Point(100, 100), cv::FONT_HERSHEY_SIMPLEX,
                1, cv::Scalar(0, 0, 255), 2);
    cv::putText(concat_mat, "depth_" + std::to_string(depth_frame.timestamp), cv::Point(100, 150),
                cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 255), 2);
    cv::imwrite(filename, concat_mat);

    // projection depth to pointcloud
    Utilities::Pointcloud pcd;
    Utilities::RgbdToPointcloud(rgb_frame.left_rgb, projected_depth, reader.GetRgbIntrinsicsLeft().as_cvmat(), pcd, 10);
    std::string pcd_filename = kVisDepthDir + "/pcd_" + ss.str() + ".obj";
    Utilities::SavePointcloudToFile(pcd_filename, pcd);

    EXPECT_TRUE(depth_frame.depth.data != nullptr);
    EXPECT_TRUE(rgb_frame.left_rgb.data != nullptr);
    EXPECT_TRUE(rgb_frame.right_rgb.data != nullptr);
  }
}

TEST(SpatialMP4Test, HeadModel_ReaderTest) {
  // test data
  auto head_model_offset = Eigen::Vector3d(-0.05057, -0.01874, 0.04309);

  // create original IMU transform
  Sophus::SE3d T_imu;
  T_imu.translation() = Eigen::Vector3d(1, 2, 3);
  T_imu.setQuaternion(Eigen::Quaterniond(0.5, 0.5, 0.5, 0.5));

  auto T_head = Utilities::ApplyHeadModel(T_imu, head_model_offset);
  auto T_imu_back = Utilities::ReleaseHeadModel(T_head, head_model_offset);

  // verify result
  EXPECT_TRUE(T_imu.translation().isApprox(T_imu_back.translation(), 1e-6));
  EXPECT_TRUE(T_imu.unit_quaternion().isApprox(T_imu_back.unit_quaternion(), 1e-6));
}

TEST(SpatialMP4Test, RgbOnly_ReaderTest) {
  if (fs::exists(kVisRgbDir)) {
    fs::remove_all(kVisRgbDir);
  }
  fs::create_directory(kVisRgbDir);

  SpatialML::Reader reader(GetVideoPath());
  ASSERT_TRUE(reader.HasRGB());
  reader.Reset();
  reader.SetReadMode(SpatialML::Reader::ReadMode::RGB_ONLY);
  while (reader.HasNext()) {
    SpatialML::rgb_frame rgb_frame;
    reader.Load(rgb_frame);
    std::cout << "rgb frame: " << rgb_frame << std::endl;
    cv::imwrite(kVisRgbDir + "/rgb_" + std::to_string(rgb_frame.timestamp) + ".png", rgb_frame.left_rgb);
    EXPECT_TRUE(rgb_frame.left_rgb.data != nullptr);
    EXPECT_TRUE(rgb_frame.right_rgb.data != nullptr);
  }
}

TEST(SpatialMP4Test, DepthOnly_ReaderTest) {
  SpatialML::Reader reader(GetVideoPath());
  ASSERT_TRUE(reader.HasDepth());
  reader.Reset();
  reader.SetReadMode(SpatialML::Reader::ReadMode::DEPTH_ONLY);
  while (reader.HasNext()) {
    SpatialML::depth_frame depth_frame;
    reader.Load(depth_frame);
    std::cout << "depth frame: " << depth_frame << std::endl;
    EXPECT_TRUE(depth_frame.depth.data != nullptr);
  }
}

TEST(SpatialMP4Test, RandomAccess_ReaderTest) {
  // spdlog::set_level(spdlog::level::debug);  // 调试模式开启DEBUG级别

  if (fs::exists(kVisRgbDir2)) {
    fs::remove_all(kVisRgbDir2);
  }
  fs::create_directory(kVisRgbDir2);

  SpatialML::RandomAccessVideoReader reader;
  if (!reader.Open(GetVideoPath(), true)) {
    std::cerr << "Failed to open video" << std::endl;
    return;
  }
  int frame_count = reader.GetFrameCount();

  // reader.Debug();
  // AVFrame* frame = reader.GetFrame(0);
  // frame = reader.GetFrame(1);
  // frame = reader.GetFrame(10);
  // frame = reader.GetFrame(800);

  for (int i = 0; i < frame_count; i += 10) {
    AVFrame* frame = reader.GetFrame(i);
    if (frame != nullptr) {
      std::pair<cv::Mat, cv::Mat> rgb_mats;
      SpatialML::FrameToBGR24(frame, rgb_mats);
      cv::putText(rgb_mats.first, std::to_string(i) + "_" + std::to_string(frame->pts), cv::Point(100, 100),
                  cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 255), 2);

      std::stringstream ss;
      ss << std::setw(4) << std::setfill('0') << i;
      cv::imwrite(kVisRgbDir2 + "/" + ss.str() + "_" + std::to_string(frame->pts) + ".png", rgb_mats.first);
    }
  }
}

TEST(SpatialMP4Test, DepthFirst_ReaderTest_Rgbd) {
  // spdlog::set_level(spdlog::level::debug); // debug mode

  if (fs::exists(kVisRgbdDir)) {
    fs::remove_all(kVisRgbdDir);
  }
  fs::create_directory(kVisRgbdDir);

  SpatialML::Reader reader(GetVideoPath());
  reader.SetReadMode(SpatialML::Reader::ReadMode::DEPTH_FIRST);
  reader.Reset();
  while (reader.HasNext()) {
    Utilities::Rgbd rgbd;
    reader.Load(rgbd);

    std::cout << "rgbd frame: \t" << rgbd.timestamp << std::endl;
    std::stringstream ss;
    ss << std::setw(4) << std::setfill('0') << reader.GetIndex();

    cv::Mat vis_projected_depth;
    Utilities::VisualizeMat(rgbd.depth, vis_projected_depth, "projected_depth", &rgbd.rgb, 0, 5, true);

    std::string filename = kVisRgbdDir + "/depth_" + ss.str() + ".png";
    cv::putText(vis_projected_depth, "depth_" + std::to_string(rgbd.timestamp), cv::Point(100, 150),
                cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 255), 2);
    cv::imwrite(filename, vis_projected_depth);

    // projection depth to pointcloud
    Utilities::Pointcloud pcd;
    Utilities::RgbdToPointcloud(rgbd.rgb, rgbd.depth, reader.GetRgbIntrinsicsLeft().as_cvmat(), pcd, 10);
    std::string pcd_filename = kVisRgbdDir + "/pcd_" + ss.str() + ".obj";
    Utilities::SavePointcloudToFile(pcd_filename, pcd);

    EXPECT_TRUE(rgbd.depth.data != nullptr);
    EXPECT_TRUE(rgbd.rgb.data != nullptr);
  }
}
