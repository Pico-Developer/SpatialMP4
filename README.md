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
- **Cross-Platform**: Full support for Linux and macOS

## 📋 System Requirements

- **Operating System**: 
  - Linux (Ubuntu 18.04+ recommended)
  - macOS (10.15+ Catalina, Xcode required)
- **Compiler**: 
  - GCC 7.0+ or Clang 6.0+ (C++17 support required)
  - Apple Clang from Xcode 11.0+ on macOS ([how-to-install](https://trac.ffmpeg.org/wiki/CompilationGuide/macOS#Xcode))
- **CMake**: 3.24.1+

## 🔧 Dependencies

The project depends on the following third-party libraries:

- [**FFmpeg**](https://github.com/FFmpeg/FFmpeg): Video encoding/decoding (libavformat, libavcodec, libswscale)
- [**OpenCV**](https://github.com/opencv/opencv): Image processing and computer vision
- [**Eigen3**](https://eigen.tuxfamily.org/index.php?title=Main_Page): Linear algebra operations
- [**Sophus**](https://github.com/strasdat/Sophus): Lie group operations for SE(3) group
- [**spdlog**](https://github.com/gabime/spdlog): High-performance logging library
- [**fmt**](https://github.com/fmtlib/fmt): Modern C++ formatting library
- [**Google Test**](https://github.com/google/googletest): Unit testing framework (optional)

## 🛠️ Build and Installation (cpp)

### 1. Clone Repository

```bash
git clone https://github.com/Pico-Developer/SpatialMP4
cd SpatialMP4
```

### 2. Build FFmpeg

Build `ffmpeg` first:

```bash
bash scripts/build_ffmpeg.sh
```

### 3. Install Dependencies

```bash
bash scripts/install_deps.sh
```

### 4. Configure and Build

```bash
mkdir build && cd build

# Configure project
cmake .. -DCMAKE_BUILD_TYPE=Release -DBUILD_PYTHON=OFF

# Build
make -j$(nproc)  # On Linux
make -j$(sysctl -n hw.ncpu)  # On macOS
```

Sometime it can be difficult to build a c++ source project. [FAQ for installation](docs/install_faq.md)
may help you. If still not working, welcome [submit a issue](https://github.com/Pico-Developer/SpatialMP4/issues).


### 5. Run Tests (Optional)

```bash
# Build with tests
cmake .. -DBUILD_TESTING=ON
make -j$(nproc)  # On Linux
make -j$(sysctl -n hw.ncpu)  # On macOS

# Run tests
cd ..
./build/test_reader
```

## 🛠️ Build and Installation (python)

### 1. Clone Repository

```bash
git clone https://github.com/Pico-Developer/SpatialMP4
cd SpatialMP4
```

### 2. Build FFmpeg

Build `ffmpeg` first:

```bash
bash scripts/build_ffmpeg.sh
```

### 3. Install Dependencies

```bash
bash scripts/install_deps.sh
```

### 4. Build and Install 

```bash
pip3 install .
```

## 📖 Usage Guide (cpp)

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

## 📚 API Reference (cpp)

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

## 📖 Usage Guide (python)


### Basic Usage Example

```python
import spatialmp4

# Create a reader
reader = spatialmp4.Reader("your_video.mp4")

# Check available streams
print("Has RGB:", reader.has_rgb())
print("Has Depth:", reader.has_depth())
print("Has Pose:", reader.has_pose())

# Set reading mode
reader.set_read_mode(spatialmp4.ReadMode.DEPTH_FIRST)

# Read frames
while reader.has_next():
    rgb_frame, depth_frame = reader.load_both()
    left_rgb = rgb_frame.left_rgb  # numpy array (H, W, 3)
    depth = depth_frame.depth      # numpy array (H, W)
    pose = rgb_frame.pose
    print("RGB timestamp:", rgb_frame.timestamp, "Pose:", pose.x, pose.y, pose.z)
```

## 📚 API Reference (python)

### Main Classes and Methods

#### `spatialmp4.Reader`
Main class for reading SpatialMP4 files.

- `Reader(filename: str)` — Create a new reader for the given file.
- `has_rgb() -> bool` — Whether the file contains RGB data.
- `has_depth() -> bool` — Whether the file contains depth data.
- `has_pose() -> bool` — Whether the file contains pose data.
- `has_audio() -> bool` — Whether the file contains audio data.
- `has_disparity() -> bool` — Whether the file contains disparity data.
- `get_duration() -> float` — Get video duration in seconds.
- `get_rgb_fps() -> float` — Get RGB stream FPS.
- `get_depth_fps() -> float` — Get depth stream FPS.
- `get_rgb_width() -> int` — Get RGB frame width.
- `get_rgb_height() -> int` — Get RGB frame height.
- `get_depth_width() -> int` — Get depth frame width.
- `get_depth_height() -> int` — Get depth frame height.
- `get_rgb_intrinsics_left() -> CameraIntrinsics` — Get left RGB camera intrinsics.
- `get_rgb_intrinsics_right() -> CameraIntrinsics` — Get right RGB camera intrinsics.
- `get_rgb_extrinsics_left() -> CameraExtrinsics` — Get left RGB camera extrinsics.
- `get_rgb_extrinsics_right() -> CameraExtrinsics` — Get right RGB camera extrinsics.
- `get_depth_intrinsics() -> CameraIntrinsics` — Get depth camera intrinsics.
- `get_depth_extrinsics() -> CameraExtrinsics` — Get depth camera extrinsics.
- `get_pose_frames() -> List[PoseFrame]` — Get all pose frames.
- `set_read_mode(mode: ReadMode)` — Set reading mode (see enums below).
- `has_next() -> bool` — Whether there is a next frame.
- `reset()` — Reset to the beginning of the file.
- `get_index() -> int` — Get current frame index.
- `get_frame_count() -> int` — Get total number of frames.
- `load_rgb() -> RGBFrame` — Load the next RGB frame.
- `load_depth(raw_head_pose: bool = False) -> DepthFrame` — Load the next depth frame.
- `load_both() -> (RGBFrame, DepthFrame)` — Load the next RGB and depth frames simultaneously.
- `load_rgbd(densify: bool = False) -> Rgbd` — Load RGBD data (for advanced use).

#### `spatialmp4.RGBFrame`
- `timestamp: float` — Frame timestamp.
- `left_rgb: np.ndarray` — Left RGB image (H, W, 3, uint8).
- `right_rgb: np.ndarray` — Right RGB image (H, W, 3, uint8).
- `pose: PoseFrame` — Associated pose data.

#### `spatialmp4.DepthFrame`
- `timestamp: float` — Frame timestamp.
- `depth: np.ndarray` — Depth image (H, W, float32, meters).
- `pose: PoseFrame` — Associated pose data.

#### `spatialmp4.PoseFrame`
- `timestamp: float` — Pose timestamp.
- `x, y, z: float` — Position.
- `qw, qx, qy, qz: float` — Quaternion orientation.
- `as_se3()` — Convert to SE(3) representation (requires Sophus/Eigen, advanced use).

#### `spatialmp4.CameraIntrinsics`
- `fx, fy, cx, cy: float` — Camera intrinsic parameters.
- `as_cvmat()` — Return as OpenCV matrix.

#### `spatialmp4.CameraExtrinsics`
- `extrinsics: np.ndarray` — 4x4 extrinsic matrix.
- `as_cvmat()` — Return as OpenCV matrix.
- `as_se3()` — Return as SE(3) (advanced use).


### Enums

#### `spatialmp4.ReadMode`
- `RGB_ONLY` — Only read RGB frames.
- `DEPTH_ONLY` — Only read depth frames.
- `DEPTH_FIRST` — Read both RGB and depth frames, depth as reference.

#### `spatialmp4.StreamType`
- `UNKNOWN` — Unknown stream type
- `AUDIO` — Audio stream
- `AUDIO_2` — Secondary audio stream
- `RGB` — RGB video stream
- `DISPARITY` — Disparity stream
- `POSE` — Pose data stream
- `DEPTH` — Depth stream


### Advanced Usage

- See [examples/python/visualize_rerun.py](./examples/python/visualize_rerun.py) and [examples/python/generate_pcd.py](./examples/python/generate_pcd.py) for advanced usage, including point cloud generation and visualization with Open3D or Rerun.
- All image and depth data are returned as NumPy arrays for easy integration with OpenCV, Open3D, PyTorch, etc.
- Camera parameters and pose data can be used for 3D reconstruction and SLAM applications.


## 🐛 Debugging and Logging

The project uses spdlog for logging:

```cpp
#include <spdlog/spdlog.h>

// Set log level
spdlog::set_level(spdlog::level::debug);

// Debug information will be automatically output in the code
```

## 📄 License

This project is licensed under the MIT. See the [LICENSE](LICENSE) file for details.

## 🤝 Contributing

Issues and Pull Requests are welcome to improve this project!

## 📞 Contact

For questions or suggestions, please contact us through GitHub Issues.
