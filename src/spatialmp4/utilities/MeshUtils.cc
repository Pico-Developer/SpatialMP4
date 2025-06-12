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

#include "MeshUtils.h"
#include <iostream>
#include <fstream>

namespace Utilities {

void MeshToFile(const float* mesh_buffer_ptr, const unsigned int* mesh_index_ptr, const unsigned int mesh_size,
                const unsigned int index_size, const std::string& output_path) {
  std::ofstream outfile(output_path, std::ios::binary);
  if (!outfile.is_open()) {
    std::cout << "VstMeshRecordingMetrics Error: Failed to open output file.";
    return;
  }

  for (size_t i = 0; i < mesh_size / 3; i++) {
    outfile << "v " << mesh_buffer_ptr[i * 3 + 0] << " " << mesh_buffer_ptr[i * 3 + 1] << " "
            << mesh_buffer_ptr[i * 3 + 2] << std::endl;
  }
  for (size_t i = 0; i < index_size / 3; i++) {
    outfile << "f " << mesh_index_ptr[i * 3 + 0] + 1 << " " << mesh_index_ptr[i * 3 + 1] + 1 << " "
            << mesh_index_ptr[i * 3 + 2] + 1 << std::endl;
  }

  // // 将 mesh_size 和 index_size 写入文件
  // outfile.write(reinterpret_cast<const char*>(&mesh_size),
  //               sizeof(unsigned int));
  // outfile.write(reinterpret_cast<const char*>(&index_size),
  //               sizeof(unsigned int));
  // // 将 mesh_buffer_ptr 写入文件
  // outfile.write(reinterpret_cast<const char*>(mesh_buffer_ptr), mesh_size);

  // // 将 mesh_index_ptr 写入文件
  // outfile.write(reinterpret_cast<const char*>(mesh_index_ptr), index_size);
  outfile.close();
}

void MeshToFile(const float* mesh_buffer_ptr, const uint16_t* mesh_index_ptr, const unsigned int mesh_size,
                const unsigned int index_size, const std::string& output_path) {
  unsigned int mesh_index[index_size];
  for (int i = 0; i < index_size; ++i) {
    mesh_index[i] = static_cast<unsigned int>(*(mesh_index_ptr + i));
  }
  MeshToFile(mesh_buffer_ptr, mesh_index, mesh_size, index_size, output_path);
}

void SeePoseToMatrix(float tx, float ty, float tz, float qw, float qx, float qy, float qz, Eigen::Matrix4f& transform) {
  Eigen::Quaternionf q(qw, qx, qy, qz);
  Eigen::Vector3f t(tx, ty, tz);
  transform = Eigen::Matrix4f::Identity();
  transform.block<3, 3>(0, 0) = q.toRotationMatrix();
  transform.block<3, 1>(0, 3) = t;
}

void TransformMesh(Eigen::Matrix4f pose, std::vector<float>& vertice_buffer) {
  Eigen::Matrix3f rotation = pose.block<3, 3>(0, 0);
  Eigen::Vector3f translation = pose.block<3, 1>(0, 3);
  for (int i = 0; i < vertice_buffer.size(); i += 3) {
    Eigen::Vector3f p(vertice_buffer[i], vertice_buffer[i + 1], vertice_buffer[i + 2]);
    Eigen::Vector3f p_transformed = rotation * p + translation;
    vertice_buffer[i] = p_transformed[0];
    vertice_buffer[i + 1] = p_transformed[1];
    vertice_buffer[i + 2] = p_transformed[2];
  }
}

}  // namespace Utilities
