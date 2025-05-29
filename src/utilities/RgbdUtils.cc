#include "RgbdUtils.h"
#include <Eigen/Eigen>
#include <opencv2/core/eigen.hpp>

namespace Utilities {

void ProjectDepthToRgb(const cv::Mat& depth, const cv::Mat& rgb, const cv::Matx33d& K_rgb_, const cv::Matx33d& K_depth_,
                       const Sophus::SE3d& T_rgb_depth, cv::Mat& projected_depth, bool debug) {
  Eigen::Matrix3d K_rgb, K_depth;
  cv::cv2eigen(K_rgb_, K_rgb);
  cv::cv2eigen(K_depth_, K_depth);

  if (debug) {
    std::cout << "K_rgb: " << K_rgb << std::endl;
    std::cout << "K_depth: " << K_depth << std::endl;
    std::cout << "rgb.size(): " << rgb.size() << std::endl;
    std::cout << "depth.size(): " << depth.size() << std::endl;
  }

  cv::Mat depth_32f;
  depth.convertTo(depth_32f, CV_32F);
  projected_depth = cv::Mat::zeros(rgb.rows, rgb.cols, CV_32FC1);

  for (int y = 0; y < depth_32f.rows; y++) {
    for (int x = 0; x < depth_32f.cols; x++) {
      double d = depth.at<float>(y, x);
      if (d < 0.1 || std::isnan(d)) {
        continue;
      }
      Eigen::Vector3d pt = Eigen::Vector3d(x, y, 1);
      Eigen::Vector3d bearing = K_depth.inverse() * pt;
      Eigen::Vector3d p_tof = bearing * d;
      Eigen::Vector3d p_rgb = T_rgb_depth.matrix().block<3, 3>(0, 0) * p_tof + T_rgb_depth.matrix().block<3, 1>(0, 3);
      if (p_rgb[2] < 0) {
        continue;
      }
      Eigen::Vector3d uv = K_rgb * (p_rgb / p_rgb[2]);
      if (uv[0] < 0 || uv[1] < 0 || uv[0] >= rgb.cols || uv[1] >= rgb.rows) {
        continue;
      }
      projected_depth.at<float>(uv[1], uv[0]) = d;
    }
  }
}

Sophus::SE3d ApplyHeadModel(const Sophus::SE3d& in, const Eigen::Vector3d& head_model_offset) {
  Eigen::Quaterniond q = in.unit_quaternion();
  Eigen::Vector3d p = in.translation();
  // 坐标系顺时针旋转90°
  Eigen::Quaterniond slam_q(q.w(), -q.y(), q.x(), q.z());
  Eigen::Vector3d slam_t(-p[1], p[0], p[2]);

  Eigen::Vector3d head = head_model_offset;
  slam_t = slam_t + slam_q * head;
  return Sophus::SE3d(slam_q, slam_t);
}

Sophus::SE3d ReleaseHeadModel(const Sophus::SE3d& in, const Eigen::Vector3d& head_model_offset) {
  Eigen::Quaterniond slam_q = in.unit_quaternion();
  Eigen::Vector3d slam_t = in.translation();
  slam_t = slam_t - slam_q * head_model_offset;

  Eigen::Quaterniond q(slam_q.w(), slam_q.y(), -slam_q.x(), slam_q.z());
  Eigen::Vector3d t(slam_t[1], -slam_t[0], slam_t[2]);
  return Sophus::SE3d(q, t);
}

void ImuToHead(const Sophus::SE3d& in, const Eigen::Vector3d& head_model_offset, Sophus::SE3d& out) {
  out = ApplyHeadModel(in, head_model_offset);
}

void HeadToImu(const Sophus::SE3d& in, const Eigen::Vector3d& head_model_offset, Sophus::SE3d& out) {
  out = ReleaseHeadModel(in, head_model_offset);
}

}  // namespace Utilities