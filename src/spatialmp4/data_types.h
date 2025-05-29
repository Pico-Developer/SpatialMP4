#pragma once
#include <iostream>
#include <vector>
#include <string>
#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <sophus/se3.hpp>
#include <opencv2/core/eigen.hpp>
#include "utilities/SyncPose.hpp"

extern "C" {
#include <libavformat/avformat.h>
#include <libavcodec/avcodec.h>
#include <libswscale/swscale.h>
#include <libavutil/pixfmt.h>
#include <libavutil/pixdesc.h>
}

namespace SpatialML {

const int DEPTH_WIDTH = 320;
const int DEPTH_HEIGHT = 240;

struct pose_frame {
  double timestamp;
  double x;
  double y;
  double z;
  double qw;
  double qx;
  double qy;
  double qz;

  friend std::ostream& operator<<(std::ostream& os, const pose_frame& frame) {
    os << "Timestamp: " << frame.timestamp << ", x: " << frame.x << ", y: " << frame.y << ", z: " << frame.z
       << ", qw: " << frame.qw << ", qx: " << frame.qx << ", qy: " << frame.qy << ", qz: " << frame.qz;
    return os;
  }

  Sophus::SE3d as_se3() const {
    Eigen::Quaterniond q(qw, qx, qy, qz);
    q.normalize();
    Eigen::Matrix3d R = q.toRotationMatrix();
    return Sophus::SE3d(R, Eigen::Vector3d(x, y, z));
  }
};

struct rgb_frame {
  double timestamp;
  cv::Mat left_rgb;
  cv::Mat right_rgb;
  pose_frame pose;  // Filled in Reader::Load()

  friend std::ostream& operator<<(std::ostream& os, const rgb_frame& frame) {
    os << "Timestamp: " << frame.timestamp << ", left_rgb height: " << frame.left_rgb.rows
       << ", left_rgb width: " << frame.left_rgb.cols << ", right_rgb height: " << frame.right_rgb.rows
       << ", right_rgb width: " << frame.right_rgb.cols;
    if (frame.pose.timestamp > 0) {
      os << " | pose: " << frame.pose;
    }
    return os;
  }
};

struct depth_frame {
  double timestamp;
  cv::Mat depth;
  pose_frame pose;  // Filled in Reader::Load()

  friend std::ostream& operator<<(std::ostream& os, const depth_frame& frame) {
    os << "Timestamp: " << frame.timestamp << ", depth height: " << frame.depth.rows
       << ", depth width: " << frame.depth.cols;
    if (frame.pose.timestamp > 0) {
      os << ", pose: " << frame.pose;
    }
    return os;
  }
};

struct camera_intrinsics {
  double fx;
  double fy;
  double cx;
  double cy;

  friend std::ostream& operator<<(std::ostream& os, const camera_intrinsics& intrinsics) {
    os << "fx: " << intrinsics.fx << ", fy: " << intrinsics.fy << ", cx: " << intrinsics.cx
       << ", cy: " << intrinsics.cy;
    return os;
  }

  cv::Matx33d as_cvmat() const { return cv::Matx33d(fx, 0, cx, 0, fy, cy, 0, 0, 1); }
};

struct camera_extrinsics {
  cv::Matx44d extrinsics;

  friend std::ostream& operator<<(std::ostream& os, const camera_extrinsics& extrinsics) {
    os << "extrinsics: " << extrinsics.extrinsics;
    return os;
  }

  cv::Matx44d as_cvmat() const { return extrinsics; }

  Sophus::SE3d as_se3() const {
    // T_Imu_Sensor
    Eigen::Matrix4d T_BS;
    cv::cv2eigen(extrinsics, T_BS);
    Eigen::Matrix3d R = T_BS.block(0, 0, 3, 3);
    Eigen::Vector3d t(T_BS(0, 3), T_BS(1, 3), T_BS(2, 3));
    Eigen::Quaterniond q(R);
    q.normalize();
    R = q.toRotationMatrix();
    return Sophus::SE3d(R, t);
  }
};

typedef enum {
  MEDIA_TYPE_UNKNOWN = -1,
  MEDIA_TYPE_AUDIO,
  MEDIA_TYPE_AUDIO_2,
  MEDIA_TYPE_RGB,
  MEDIA_TYPE_DISPARITY,
  MEDIA_TYPE_POSE,
  MEDIA_TYPE_DEPTH,
} StreamType;

}  // namespace SpatialML
