#pragma once

/**
 * Easy to use opencv utilities.
 *
 * Usage:
 *  1. Utilities::OpencvTypeToString(CV_8UC1);
 *  2. Utilities::PrintOpencvMat(mat, "mat_name");
 *  3. Utilities::VisualizeMat(mat);
 *  4. Utilities::DumpMat(mat, "save_path.bin");
 *  5. Utilities::LoadMat("save_path.bin", mat);
 */

#include <iostream>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

namespace Utilities {

std::string OpencvTypeToString(int type);
void PrintOpencvMat(const cv::Mat& mat, const std::string& name);
void VisualizeMat(const cv::Mat depth, cv::Mat& vis_depth, std::string name, cv::Mat* image_ptr = nullptr,
                  double minVal = -1, double maxVal = -1, bool check_size = false);
/**
 * DumpMat: dump mat to binary file
 * @param mat: the mat to be dumped
 * @param filename: the filename of the dumped file
 *
 * LoadMat in python:
 *
 *   import cv2
 *   import numpy as np
 *   def load_mat(filename):
 *     fs = cv2.FileStorage(filename, cv2.FILE_STORAGE_READ)
 *     if not fs.isOpened():
 *       print("Error: Could not open the file for reading.")
 *       return
 *     else:
 *       loaded_mat = fs.getNode("image").mat()
 *       fs.release()
 *       return loaded_mat
 */
void DumpMat(const cv::Mat mat, std::string filename);

void LoadMat(const std::string filename, cv::Mat& mat);

cv::Mat ConcatenateMat(const std::vector<cv::Mat>& mats, const std::vector<std::string>& mats_name,
                       const std::string& save_path = "");

}  // namespace Utilities
