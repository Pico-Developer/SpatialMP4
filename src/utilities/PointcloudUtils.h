#pragma once

/**
 * Easy to use Pointcloud utilities.
 *
 * Usage:
 *  1. Utilities::Pointcloud;
 *  2. Utilities::RgbdToPointcloud(rgb, depth, K_rgb, pcd);
 *  3. Utilities::SavePointcloudToFile(savePath, pcd);
 */

#include <iostream>
#include <fstream>
#include <vector>
#include <Eigen/Core>
#include <opencv2/opencv.hpp>

namespace Utilities {

class Pointcloud {
 public:
  Pointcloud();
  Pointcloud(const std::vector<Eigen::Vector3d>& points);
  ~Pointcloud();

  Pointcloud& Clear();

  inline bool IsEmpty() const { return !HasPoints(); }

  Pointcloud& operator+=(const Pointcloud& cloud);

  inline Pointcloud operator+(const Pointcloud& cloud) const { return (Pointcloud(*this) += cloud); }

  /// Returns 'true' if the point cloud contains points.
  inline bool HasPoints() const { return points_.size() > 0; }

  /// Returns `true` if the point cloud contains point normals.
  inline bool HasNormals() const { return points_.size() > 0 && normals_.size() == points_.size(); }

  /// Returns `true` if the point cloud contains point colors.
  inline bool HasColors() const { return points_.size() > 0 && colors_.size() == points_.size(); }

  /// Returns 'true' if the point cloud contains per-point covariance matrix.
  inline bool HasCovariances() const { return !points_.empty() && covariances_.size() == points_.size(); }

  // /// Returns 'true' if the point cloud contains point semantic labels.
  // inline bool HasSemanticLabels() const {
  //     return !points_.empty() && semantic_labels_.size() == points_.size();
  // }

  /// Returns 'true' if the point cloud contains rays.
  inline bool HasRays() const { return !points_.empty() && rays_.size() == points_.size(); }

  /// Normalize point normals to length 1.
  Pointcloud& NormalizeNormals();

 public:
  /// Points coordinates.
  std::vector<Eigen::Vector3d> points_;
  /// Points normals.
  std::vector<Eigen::Vector3d> normals_;
  /// RGB colors of points.
  std::vector<Eigen::Vector3d> colors_;
  /// Covariance Matrix for each point
  std::vector<Eigen::Matrix3d> covariances_;
  /// Depth confidence of points.
  std::vector<float> depth_confidences_;
  /// Ray length.
  std::vector<float> rays_;
  // /// Semantic labels of points.
  // std::vector<SemanticLabel> semantic_labels_;
  // /// Semantic scores of points.
  // std::vector<float> semantic_scores_;
};

void RgbdToPointcloud(const cv::Mat& rgb, const cv::Mat& depth, const cv::Matx33d& K_rgb_, Pointcloud& pcd,
                      float max_depth = -1);

bool endswith(const std::string& input_str, const std::string& suffix);

void SavePointcloudToFile(const std::string savePath, const Pointcloud& pcd);

void ProjectPointcloudToRgb(const Pointcloud& pcd, const cv::Mat& rgb, const Eigen::Matrix3d& K_rgb, cv::Mat& vis_rgb);

}  // namespace Utilities
