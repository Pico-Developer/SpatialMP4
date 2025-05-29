/**
 * Easy to use Pointcloud utilities.
 *
 * Usage:
 *  1. Utilities::Pointcloud;
 *  2. Utilities::RgbdToPointcloud(rgb, depth, K_rgb, pcd);
 *  3. Utilities::SavePointcloudToFile(savePath, pcd);
 */
#include "PointcloudUtils.h"
#include <Eigen/Eigen>
#include <opencv2/core/eigen.hpp>

namespace Utilities {

Pointcloud::Pointcloud() {}

Pointcloud::Pointcloud(const std::vector<Eigen::Vector3d>& points) : points_(points) {}

Pointcloud::~Pointcloud() {}

Pointcloud& Pointcloud::Clear() {
  points_.clear();
  normals_.clear();
  colors_.clear();
  covariances_.clear();
  rays_.clear();
  // semantic_labels_.clear();
  return *this;
}

Pointcloud& Pointcloud::operator+=(const Pointcloud& cloud) {
  // We do not use std::vector::insert to combine std::vector because it will
  // crash if the pointcloud is added to itself.
  if (cloud.IsEmpty()) return (*this);
  size_t old_vert_num = points_.size();
  size_t add_vert_num = cloud.points_.size();
  size_t new_vert_num = old_vert_num + add_vert_num;
  if ((!HasPoints() || HasNormals()) && cloud.HasNormals()) {
    normals_.resize(new_vert_num);
    for (size_t i = 0; i < add_vert_num; i++) normals_[old_vert_num + i] = cloud.normals_[i];
  } else {
    normals_.clear();
  }
  if ((!HasPoints() || HasColors()) && cloud.HasColors()) {
    colors_.resize(new_vert_num);
    for (size_t i = 0; i < add_vert_num; i++) colors_[old_vert_num + i] = cloud.colors_[i];
  } else {
    colors_.clear();
  }
  if ((!HasPoints() || HasCovariances()) && cloud.HasCovariances()) {
    covariances_.resize(new_vert_num);
    for (size_t i = 0; i < add_vert_num; i++) covariances_[old_vert_num + i] = cloud.covariances_[i];
  } else {
    covariances_.clear();
  }
  // if ((!HasPoints() || HasSemanticLabels()) && cloud.HasSemanticLabels()) {
  //     semantic_labels_.resize(new_vert_num);
  //     for (size_t i = 0; i < add_vert_num; i++)
  //         semantic_labels_[old_vert_num + i] = cloud.semantic_labels_[i];
  // } else {
  //     semantic_labels_.clear();
  // }
  if ((!HasPoints() || HasRays()) && cloud.HasRays()) {
    rays_.resize(new_vert_num);
    for (size_t i = 0; i < add_vert_num; i++) rays_[old_vert_num + i] = cloud.rays_[i];
  } else {
    rays_.clear();
  }
  points_.resize(new_vert_num);
  for (size_t i = 0; i < add_vert_num; i++) points_[old_vert_num + i] = cloud.points_[i];
  return (*this);
}

void RgbdToPointcloud(const cv::Mat& rgb, const cv::Mat& depth, const cv::Matx33d& K_rgb_, Pointcloud& pcd,
                      float max_depth) {
  CV_Assert(rgb.size() == depth.size());
  Eigen::Matrix3d K_rgb;
  cv::cv2eigen(K_rgb_, K_rgb);

  pcd.Clear();
  int width = rgb.cols;
  int height = rgb.rows;
  for (int v = 0; v < height; ++v) {
    for (int u = 0; u < width; ++u) {
      float d = depth.at<float>(v, u);
      if (d < 0.1 || std::isnan(d)) {
        continue;
      }
      if (max_depth > 0 and d > max_depth) {
        continue;
      }
      Eigen::Vector3d pt = Eigen::Vector3d(u, v, 1);
      Eigen::Vector3d point = K_rgb.inverse() * pt * d;
      pcd.points_.push_back(point);
      cv::Vec3b color = rgb.at<cv::Vec3b>(v, u);
      Eigen::Vector3d color_d(color[0] / 255., color[1] / 255., color[2] / 255.);
      pcd.colors_.push_back(color_d);
    }
  }
}

bool endswith(const std::string& input_str, const std::string& suffix) {
  if (input_str.size() >= suffix.size()) {
    return input_str.substr(input_str.size() - suffix.size()) == suffix;
  }
  return false;
}

void SavePointcloudToFile(const std::string savePath, const Pointcloud& pcd) {
  std::ofstream f_point_cloud;
  f_point_cloud.open(savePath);

  for (size_t i = 0; i < pcd.points_.size(); ++i) {
    if (endswith(savePath, ".obj")) {
      f_point_cloud << "v ";
    }
    CV_Assert(endswith(savePath, ".txt") | endswith(savePath, ".obj"));

    f_point_cloud << pcd.points_[i][0] << " " << pcd.points_[i][1] << " " << pcd.points_[i][2] << " "
                  << int(pcd.colors_[i][0] * 255) << " " << int(pcd.colors_[i][1] * 255) << " "
                  << int(pcd.colors_[i][2] * 255) << " " << std::endl;
  }
  f_point_cloud.close();
}

void ProjectPointcloudToRgb(const Pointcloud& pcd, const cv::Mat& rgb, const Eigen::Matrix3d& K_rgb, cv::Mat& vis_rgb) {
  // TODO: Implement this function
}

}  // namespace Utilities
