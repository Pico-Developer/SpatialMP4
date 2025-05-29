# SpatialMP4

A C++/Python toolkit for processing SpatialMP4 format, supporting reading and processing spatial video files containing RGB, depth, pose, and audio data.

## 🚀 Features

- **Multi-modal Data Support**: Simultaneously process RGB images, depth maps, pose data, and audio
- **Stereo Vision**: Support for left and right eye RGB image data
- **High Performance**: Efficient video decoding based on FFmpeg
- **Flexible Reading Modes**: Support for RGB-only, Depth-only, and Depth-first reading modes
- **Random Access**: Support for random access to video frames and keyframe indexing
- **3D Reconstruction**: Built-in point cloud generation and RGBD data processing
- **Camera Calibration**: Support for reading and applying intrinsic and extrinsic parameters
- **Visualization Tools**: Rich data visualization and debugging capabilities

## 📋 System Requirements

- **Operating System**: Linux (Ubuntu 18.04+ recommended)
- **Compiler**: GCC 7.0+ or Clang 6.0+ (C++17 support required)
- **CMake**: 3.22.1+

## 🔧 Dependencies

The project depends on the following third-party libraries:

- **FFmpeg**: Video encoding/decoding (libavformat, libavcodec, libswscale)
- **OpenCV**: Image processing and computer vision
- **Eigen3**: Linear algebra operations
- **Sophus**: Lie group operations for SE(3) group
- **spdlog**: High-performance logging library
- **fmt**: Modern C++ formatting library
- **Google Test**: Unit testing framework (optional)

## 🛠️ Build and Installation

### 1. Clone Repository

```bash
git clone <repository-url>
cd SpatialMP4
```

### 2. Create Build Directory

Build `ffmpeg` first.

```bash
bash scripts/build_ffmpeg.sh
```

```bash
mkdir build && cd build
```

### 3. Configure and Build

```bash
# Configure project
cmake .. -DCMAKE_BUILD_TYPE=Release

# Build
make -j$(nproc)
```

### 4. Run Tests (Optional)

```bash
# Build with tests
cmake .. -DBUILD_TESTING=ON
make -j$(nproc)

# Run tests
./test_reader
```

## 📖 Usage Guide

### Basic Usage

```cpp
#include "spatialmp4/reader.h"
#include "spatialmp4/data_types.h"

// Create reader
SpatialML::Reader reader("path/to/your/spatial.mp4");

// Check data types
if (reader.HasRGB()) {
    std::cout << "Contains RGB data" << std::endl;
}
if (reader.HasDepth()) {
    std::cout << "Contains depth data" << std::endl;
}
if (reader.HasPose()) {
    std::cout << "Contains pose data" << std::endl;
}

// Get camera parameters
auto rgb_intrinsics = reader.GetRgbIntrinsicsLeft();
auto depth_intrinsics = reader.GetDepthIntrinsics();
```

### Reading RGB and Depth Data

```cpp
// Set reading mode
reader.SetReadMode(SpatialML::Reader::ReadMode::DEPTH_FIRST);
reader.Reset();

while (reader.HasNext()) {
    SpatialML::rgb_frame rgb_frame;
    SpatialML::depth_frame depth_frame;
    
    // Read RGB and depth frames simultaneously
    reader.Load(rgb_frame, depth_frame);
    
    // Process data
    cv::Mat left_rgb = rgb_frame.left_rgb;
    cv::Mat right_rgb = rgb_frame.right_rgb;
    cv::Mat depth = depth_frame.depth;
    
    std::cout << "RGB timestamp: " << rgb_frame.timestamp << std::endl;
    std::cout << "Depth timestamp: " << depth_frame.timestamp << std::endl;
}
```

### RGB-Only Reading

```cpp
reader.SetReadMode(SpatialML::Reader::ReadMode::RGB_ONLY);
reader.Reset();

while (reader.HasNext()) {
    SpatialML::rgb_frame rgb_frame;
    reader.Load(rgb_frame);
    
    // Process RGB data
    cv::imshow("Left RGB", rgb_frame.left_rgb);
    cv::imshow("Right RGB", rgb_frame.right_rgb);
    cv::waitKey(1);
}
```

### Pose Data Processing

```cpp
// Get all pose data
auto pose_frames = reader.GetPoseFrames();

for (const auto& pose : pose_frames) {
    // Convert to SE(3) representation
    Sophus::SE3d se3_pose = pose.as_se3();
    
    // Get rotation and translation
    Eigen::Matrix3d rotation = se3_pose.rotationMatrix();
    Eigen::Vector3d translation = se3_pose.translation();
    
    std::cout << "Pose timestamp: " << pose.timestamp << std::endl;
    std::cout << "Position: " << translation.transpose() << std::endl;
}
```

### Depth Projection to RGB

```cpp
#include "utilities/RgbdUtils.h"

// Get camera parameters
auto rgb_intrinsics = reader.GetRgbIntrinsicsLeft().as_cvmat();
auto depth_intrinsics = reader.GetDepthIntrinsics().as_cvmat();

// Calculate transformation matrix
auto T_I_Srgb = reader.GetRgbExtrinsicsLeft().as_se3();
auto T_I_Stof = reader.GetDepthExtrinsics().as_se3();
auto T_Srgb_Stof = T_I_Srgb.inverse() * T_I_Stof;

// Project depth to RGB
cv::Mat projected_depth;
Utilities::ProjectDepthToRgb(depth_frame.depth, rgb_frame.left_rgb, 
                           rgb_intrinsics, depth_intrinsics, 
                           T_Srgb_Stof, projected_depth);
```

### Point Cloud Generation

```cpp
#include "utilities/PointcloudUtils.h"

// Generate point cloud from RGBD data
Utilities::Pointcloud pcd;
Utilities::RgbdToPointcloud(rgb_frame.left_rgb, projected_depth, 
                          rgb_intrinsics, pcd, 10.0f);

// Save point cloud
Utilities::SavePointcloudToFile("output.obj", pcd);
```

## 📚 API Reference

### SpatialML::Reader

Main SpatialMP4 file reader class.

#### Constructor
```cpp
Reader(const std::string& filename)
```

#### Data Check Methods
```cpp
bool HasRGB() const;        // Whether contains RGB data
bool HasDepth() const;      // Whether contains depth data  
bool HasPose() const;       // Whether contains pose data
bool HasAudio() const;      // Whether contains audio data
bool HasDisparity() const;  // Whether contains disparity data
```

#### Parameter Getter Methods
```cpp
camera_intrinsics GetRgbIntrinsicsLeft() const;   // Left RGB camera intrinsics
camera_intrinsics GetRgbIntrinsicsRight() const;  // Right RGB camera intrinsics
camera_extrinsics GetRgbExtrinsicsLeft() const;   // Left RGB camera extrinsics
camera_extrinsics GetRgbExtrinsicsRight() const;  // Right RGB camera extrinsics
camera_intrinsics GetDepthIntrinsics() const;     // Depth camera intrinsics
camera_extrinsics GetDepthExtrinsics() const;     // Depth camera extrinsics
```

#### Reading Control Methods
```cpp
void SetReadMode(ReadMode mode);  // Set reading mode
bool HasNext() const;             // Whether has next frame
void Reset();                     // Reset to beginning
int GetIndex() const;             // Get current index
```

#### Data Loading Methods
```cpp
void Load(rgb_frame& rgb_frame);                              // Load RGB frame
void Load(depth_frame& depth_frame);                          // Load depth frame  
void Load(rgb_frame& rgb_frame, depth_frame& depth_frame);    // Load RGB and depth frames simultaneously
```

### Data Structures

#### rgb_frame
```cpp
struct rgb_frame {
    double timestamp;     // Timestamp
    cv::Mat left_rgb;     // Left eye RGB image
    cv::Mat right_rgb;    // Right eye RGB image
    pose_frame pose;      // Corresponding pose data
};
```

#### depth_frame
```cpp
struct depth_frame {
    double timestamp;     // Timestamp
    cv::Mat depth;        // Depth image
    pose_frame pose;      // Corresponding pose data
};
```

#### pose_frame
```cpp
struct pose_frame {
    double timestamp;     // Timestamp
    double x, y, z;       // Position
    double qw, qx, qy, qz; // Quaternion rotation
    
    Sophus::SE3d as_se3() const;  // Convert to SE(3) representation
};
```

## 🔍 Utility Functions

### Image Processing (OpencvUtils)
- `VisualizeMat()`: Visualize matrix data
- `DumpMat()` / `LoadMat()`: Save/load matrices
- `ConcatenateMat()`: Concatenate multiple images

### RGBD Processing (RgbdUtils)  
- `ProjectDepthToRgb()`: Project depth to RGB
- `RgbdToPointcloud()`: Convert RGBD to point cloud

### Point Cloud Processing (PointcloudUtils)
- `SavePointcloudToFile()`: Save point cloud files
- Support for OBJ format output

## 🐛 Debugging and Logging

The project uses spdlog for logging:

```cpp
#include <spdlog/spdlog.h>

// Set log level
spdlog::set_level(spdlog::level::debug);

// Debug information will be automatically output in the code
```

## 📄 License

This project is licensed under the GNU Lesser General Public License v2.1. See the [LICENSE](LICENSE) file for details.

## 🤝 Contributing

Issues and Pull Requests are welcome to improve this project!

## 📞 Contact

For questions or suggestions, please contact us through GitHub Issues.
