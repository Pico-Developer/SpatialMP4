#pragma once
#include <iostream>
#include <fstream>

namespace Utilities {

void MeshToFile(const float* mesh_buffer_ptr, const unsigned int* mesh_index_ptr, const unsigned int mesh_size,
                const unsigned int index_size, const std::string& output_path);

void MeshToFile(const float* mesh_buffer_ptr, const uint16_t* mesh_index_ptr, const unsigned int mesh_size,
                const unsigned int index_size, const std::string& output_path);

void SeePoseToMatrix(float tx, float ty, float tz, float qw, float qx, float qy, float qz, Eigen::Matrix4f& transform);

void TransformMesh(Eigen::Matrix4f pose, std::vector<float>& vertice_buffer);

}  // namespace Utilities
