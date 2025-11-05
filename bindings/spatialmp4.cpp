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

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <opencv2/core/core.hpp>
#include "spatialmp4/reader.h"
#include "spatialmp4/data_types.h"
#include "spatialmp4/utils.h"
#include "spatialmp4/version.h"
#include "spatialmp4/utilities/RgbdUtils.h"

namespace py = pybind11;

// Helper function to convert cv::Mat to numpy array
py::array_t<uint8_t> mat_to_numpy(const cv::Mat &mat) {
  if (mat.empty()) {
    return py::array_t<uint8_t>();
  }

  py::array_t<uint8_t> array({mat.rows, mat.cols, mat.channels()});
  std::memcpy(array.mutable_data(), mat.data, mat.total() * mat.elemSize());
  return array;
}

// Helper function to convert depth cv::Mat to numpy array
py::array_t<float> depth_mat_to_numpy(const cv::Mat &mat) {
  if (mat.empty()) {
    return py::array_t<float>();
  }

  py::array_t<float> array({mat.rows, mat.cols});
  std::memcpy(array.mutable_data(), mat.data, mat.total() * mat.elemSize());
  return array;
}

PYBIND11_MODULE(spatialmp4, m) {
  m.doc() = "SpatialMP4 Python Bindings";

  // 添加版本信息
  m.attr("__version__") = SpatialML::GetVersion();
  m.def("get_version", &SpatialML::GetVersion, "Get the version string");
  m.def("get_major_version", &SpatialML::GetMajorVersion, "Get the major version number");
  m.def("get_minor_version", &SpatialML::GetMinorVersion, "Get the minor version number");
  m.def("get_patch_version", &SpatialML::GetPatchVersion, "Get the patch version number");

  // Bind StreamType enum
  py::enum_<SpatialML::StreamType>(m, "StreamType")
      .value("UNKNOWN", SpatialML::MEDIA_TYPE_UNKNOWN)
      .value("AUDIO", SpatialML::MEDIA_TYPE_AUDIO)
      .value("AUDIO_2", SpatialML::MEDIA_TYPE_AUDIO_2)
      .value("RGB", SpatialML::MEDIA_TYPE_RGB)
      .value("DISPARITY", SpatialML::MEDIA_TYPE_DISPARITY)
      .value("POSE", SpatialML::MEDIA_TYPE_POSE)
      .value("DEPTH", SpatialML::MEDIA_TYPE_DEPTH)
      .export_values();

  // Bind pose_frame struct
  py::class_<SpatialML::pose_frame>(m, "PoseFrame")
      .def(py::init<>())
      .def_readwrite("timestamp", &SpatialML::pose_frame::timestamp)
      .def_readwrite("x", &SpatialML::pose_frame::x)
      .def_readwrite("y", &SpatialML::pose_frame::y)
      .def_readwrite("z", &SpatialML::pose_frame::z)
      .def_readwrite("qw", &SpatialML::pose_frame::qw)
      .def_readwrite("qx", &SpatialML::pose_frame::qx)
      .def_readwrite("qy", &SpatialML::pose_frame::qy)
      .def_readwrite("qz", &SpatialML::pose_frame::qz)
      .def("as_se3", &SpatialML::pose_frame::as_se3)
      .def("__repr__", [](const SpatialML::pose_frame &p) {
        std::ostringstream ss;
        ss << p;
        return ss.str();
      });

  // Bind rgb_frame struct
  py::class_<SpatialML::rgb_frame>(m, "RGBFrame")
      .def(py::init<>())
      .def_readwrite("timestamp", &SpatialML::rgb_frame::timestamp)
      .def_property(
          "left_rgb", [](const SpatialML::rgb_frame &f) { return mat_to_numpy(f.left_rgb); },
          [](SpatialML::rgb_frame &f, py::array_t<uint8_t> arr) {
            // TODO: implement setter if needed
          })
      .def_property(
          "right_rgb", [](const SpatialML::rgb_frame &f) { return mat_to_numpy(f.right_rgb); },
          [](SpatialML::rgb_frame &f, py::array_t<uint8_t> arr) {
            // TODO: implement setter if needed
          })
      .def_readwrite("pose", &SpatialML::rgb_frame::pose)
      .def("__repr__", [](const SpatialML::rgb_frame &f) {
        std::ostringstream ss;
        ss << f;
        return ss.str();
      });

  // Bind depth_frame struct
  py::class_<SpatialML::depth_frame>(m, "DepthFrame")
      .def(py::init<>())
      .def_readwrite("timestamp", &SpatialML::depth_frame::timestamp)
      .def_property(
          "depth", [](const SpatialML::depth_frame &f) { return depth_mat_to_numpy(f.depth); },
          [](SpatialML::depth_frame &f, py::array_t<float> arr) {
            // TODO: implement setter if needed
          })
      .def_readwrite("pose", &SpatialML::depth_frame::pose)
      .def("__repr__", [](const SpatialML::depth_frame &f) {
        std::ostringstream ss;
        ss << f;
        return ss.str();
      });

  // Bind camera_intrinsics struct
  py::class_<SpatialML::camera_intrinsics>(m, "CameraIntrinsics")
      .def(py::init<>())
      .def_readwrite("fx", &SpatialML::camera_intrinsics::fx)
      .def_readwrite("fy", &SpatialML::camera_intrinsics::fy)
      .def_readwrite("cx", &SpatialML::camera_intrinsics::cx)
      .def_readwrite("cy", &SpatialML::camera_intrinsics::cy)
      .def("as_cvmat", &SpatialML::camera_intrinsics::as_cvmat)
      .def("__repr__", [](const SpatialML::camera_intrinsics &i) {
        std::ostringstream ss;
        ss << i;
        return ss.str();
      });

  // Bind camera_extrinsics struct
  py::class_<SpatialML::camera_extrinsics>(m, "CameraExtrinsics")
      .def(py::init<>())
      .def_readwrite("extrinsics", &SpatialML::camera_extrinsics::extrinsics)
      .def("as_cvmat", &SpatialML::camera_extrinsics::as_cvmat)
      .def("as_se3", &SpatialML::camera_extrinsics::as_se3)
      .def("__repr__", [](const SpatialML::camera_extrinsics &e) {
        std::ostringstream ss;
        ss << e;
        return ss.str();
      });

  // Bind Reader class
  py::class_<SpatialML::Reader>(m, "Reader")
      .def(py::init<const std::string &>())
      .def("has_rgb", &SpatialML::Reader::HasRGB)
      .def("has_depth", &SpatialML::Reader::HasDepth)
      .def("has_pose", &SpatialML::Reader::HasPose)
      .def("has_audio", &SpatialML::Reader::HasAudio)
      .def("has_disparity", &SpatialML::Reader::HasDisparity)
      .def("get_start_timestamp", &SpatialML::Reader::GetStartTimestamp)
      .def("get_duration", &SpatialML::Reader::GetDuration)
      .def("get_rgb_fps", &SpatialML::Reader::GetRgbFPS)
      .def("get_rgb_width", &SpatialML::Reader::GetRgbWidth)
      .def("get_rgb_height", &SpatialML::Reader::GetRgbHeight)
      .def("get_rgb_intrinsics_left", &SpatialML::Reader::GetRgbIntrinsicsLeft)
      .def("get_rgb_extrinsics_left", &SpatialML::Reader::GetRgbExtrinsicsLeft)
      .def("get_rgb_intrinsics_right", &SpatialML::Reader::GetRgbIntrinsicsRight)
      .def("get_rgb_extrinsics_right", &SpatialML::Reader::GetRgbExtrinsicsRight)
      .def("get_depth_fps", &SpatialML::Reader::GetDepthFPS)
      .def("get_depth_width", &SpatialML::Reader::GetDepthWidth)
      .def("get_depth_height", &SpatialML::Reader::GetDepthHeight)
      .def("get_depth_intrinsics", &SpatialML::Reader::GetDepthIntrinsics)
      .def("get_depth_extrinsics", &SpatialML::Reader::GetDepthExtrinsics)
      .def("get_pose_frames", &SpatialML::Reader::GetPoseFrames)
      .def("get_rgb_keyframe_index", &SpatialML::Reader::GetRgbKeyframeIndex)
      .def("get_depth_keyframe_index", &SpatialML::Reader::GetDepthKeyframeIndex)
      .def("is_rgb_distorted", &SpatialML::Reader::IsRgbDistorted)
      .def("get_rgb_distortion_model", &SpatialML::Reader::GetRgbDistortionModel)
      .def("get_rgb_distortion_params_left", &SpatialML::Reader::GetRgbDistortionParamsLeft)
      .def("get_rgb_distortion_params_right", &SpatialML::Reader::GetRgbDistortionParamsRight)
      .def("get_depth_distortion_model", &SpatialML::Reader::GetDepthDistortionModel)
      .def("get_depth_distortion_params", &SpatialML::Reader::GetDepthDistortionParams)
      .def("get_rgb_timebase", &SpatialML::Reader::GetRgbTimebase)
      .def("get_depth_timebase", &SpatialML::Reader::GetDepthTimebase)
      .def("set_read_mode", &SpatialML::Reader::SetReadMode)
      .def("has_next", &SpatialML::Reader::HasNext)
      .def("load_rgb",
           [](SpatialML::Reader &self) {
             SpatialML::rgb_frame frame;
             self.Load(frame);
             return frame;
           })
      .def(
          "load_depth",
          [](SpatialML::Reader &self, bool raw_head_pose) {
            SpatialML::depth_frame frame;
            self.Load(frame, raw_head_pose);
            return frame;
          },
          py::arg("raw_head_pose") = false)
      .def("load_both",
           [](SpatialML::Reader &self) {
             SpatialML::rgb_frame rgb_frame;
             SpatialML::depth_frame depth_frame;
             self.Load(rgb_frame, depth_frame);
             return py::make_tuple(rgb_frame, depth_frame);
           })
      .def("load_rgbd",
           [](SpatialML::Reader &self, bool densify = false) {
             Utilities::Rgbd rgbd;
             self.Load(rgbd, densify);
             return rgbd;
           })
      .def("reset", &SpatialML::Reader::Reset)
      .def("get_index", &SpatialML::Reader::GetIndex)
      .def("get_frame_count", &SpatialML::Reader::GetFrameCount);

  // Bind ReadMode enum
  py::enum_<SpatialML::Reader::ReadMode>(m, "ReadMode")
      .value("RGB_ONLY", SpatialML::Reader::RGB_ONLY)
      .value("DEPTH_ONLY", SpatialML::Reader::DEPTH_ONLY)
      .value("DEPTH_FIRST", SpatialML::Reader::DEPTH_FIRST)
      .export_values();

  // Bind RandomAccessVideoReader class
  py::class_<SpatialML::RandomAccessVideoReader>(m, "RandomAccessVideoReader")
      .def(py::init<>())
      .def("open", &SpatialML::RandomAccessVideoReader::Open)
      .def("get_frame",
           [](SpatialML::RandomAccessVideoReader &self, int64_t frame_number) -> py::object {
             AVFrame *frame = self.GetFrame(frame_number);
             if (!frame) {
               return py::none();
             }
             std::pair<cv::Mat, cv::Mat> rgb_mats;
             SpatialML::FrameToBGR24(frame, rgb_mats);
             return py::make_tuple(mat_to_numpy(rgb_mats.first), mat_to_numpy(rgb_mats.second));
           })
      .def("get_frame_count", &SpatialML::RandomAccessVideoReader::GetFrameCount)
      .def("debug", &SpatialML::RandomAccessVideoReader::Debug);

  // Bind Utilities::Rgbd struct
  py::class_<Utilities::Rgbd>(m, "Rgbd")
      .def(py::init<>())
      .def_readwrite("timestamp", &Utilities::Rgbd::timestamp)
      .def_property(
          "rgb", [](const Utilities::Rgbd &f) { return mat_to_numpy(f.rgb); },
          [](Utilities::Rgbd &f, py::array_t<uint8_t> arr) {
            // TODO: implement setter if needed
          })
      .def_property(
          "depth", [](const Utilities::Rgbd &f) { return depth_mat_to_numpy(f.depth); },
          [](Utilities::Rgbd &f, py::array_t<float> arr) {
            // TODO: implement setter if needed
          })
      .def_property_readonly("T_W_S",
                             [](const Utilities::Rgbd &f) {
                               // Expose as Eigen::Matrix4d for now
                               Eigen::Matrix4d T = f.T_W_S.matrix();
                               return T;
                             })
      .def("__repr__", [](const Utilities::Rgbd &f) {
        std::ostringstream ss;
        ss << "Rgbd(timestamp=" << f.timestamp << ", rgb shape=[" << f.rgb.rows << "," << f.rgb.cols << ","
           << f.rgb.channels() << "]";
        ss << ", depth shape=[" << f.depth.rows << "," << f.depth.cols << "]";
        ss << ")";
        return ss.str();
      });

  m.def(
      "head_to_imu",
      [](py::array_t<double> head_pose_array, py::array_t<double> head_model_offset_array) {
        py::buffer_info pose_info = head_pose_array.request();
        if (pose_info.ndim != 2 || pose_info.shape[0] != 4 || pose_info.shape[1] != 4) {
          throw std::invalid_argument("head_pose must be a 4x4 matrix");
        }
        py::buffer_info offset_info = head_model_offset_array.request();
        if (offset_info.ndim != 1 || offset_info.shape[0] != 3) {
          throw std::invalid_argument("head_model_offset must be a 3-element vector");
        }

        Eigen::Map<const Eigen::Matrix<double, 4, 4, Eigen::RowMajor>> head_pose_map(
            static_cast<double *>(pose_info.ptr));
        Eigen::Map<const Eigen::Vector3d> head_model_offset_map(static_cast<double *>(offset_info.ptr));

        Eigen::Matrix4d head_pose_matrix = head_pose_map;
        Eigen::Vector3d head_model_offset = head_model_offset_map;

        Eigen::Matrix3d R = head_pose_matrix.block<3, 3>(0, 0);
        Eigen::Vector3d t = head_pose_matrix.block<3, 1>(0, 3);
        Eigen::Quaterniond q(R);
        q.normalize();
        Sophus::SO3d so3(q);
        Sophus::SE3d head_pose(so3, t);
        Sophus::SE3d imu_pose;
        Utilities::HeadToImu(head_pose, head_model_offset, imu_pose);

        Eigen::Matrix<double, 4, 4, Eigen::RowMajor> result = imu_pose.matrix();
        py::array_t<double> output({4, 4});
        std::memcpy(output.mutable_data(), result.data(), sizeof(double) * 16);
        return output;
      },
      py::arg("head_pose"), py::arg("head_model_offset"));
}
