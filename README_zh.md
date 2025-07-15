# SpatialMP4

一个用于处理SpatialMP4格式的C++/Python工具包，支持读取和处理包含RGB、深度、位姿和音频数据的空间视频文件。

## 🚀 功能特性

- **多模态数据支持**: 同时处理RGB图像、深度图、位姿数据和音频
- **立体视觉**: 支持左右眼RGB图像数据
- **高性能**: 基于FFmpeg的高效视频解码
- **灵活的读取模式**: 支持RGB-only、Depth-only和Depth-first等多种读取模式
- **随机访问**: 支持视频帧的随机访问和关键帧索引
- **3D重建**: 内置点云生成和RGBD数据处理功能
- **相机标定**: 支持内参和外参的读取与应用
- **可视化工具**: 丰富的数据可视化和调试功能
- **跨平台支持**: 完整支持Linux和macOS系统

## 📋 系统要求

- **操作系统**: 
  - Linux (推荐Ubuntu 18.04+)
  - macOS (10.15+ Catalina，需要安装Xcode)
- **编译器**: 
  - GCC 7.0+ 或 Clang 6.0+ (支持C++17)
  - macOS上需要Xcode 11.0+的Apple Clang ([how-to-install](https://trac.ffmpeg.org/wiki/CompilationGuide/macOS#Xcode))
- **CMake**: 3.24.1+

## 🔧 依赖库

项目依赖以下第三方库：

- [**FFmpeg**](https://github.com/FFmpeg/FFmpeg): 视频编解码 (libavformat, libavcodec, libswscale)
- [**OpenCV**](https://github.com/opencv/opencv): 图像处理和计算机视觉
- [**Eigen3**](https://eigen.tuxfamily.org/index.php?title=Main_Page): 线性代数运算
- [**Sophus**](https://github.com/strasdat/Sophus): SE(3)群的李群操作
- [**spdlog**](https://github.com/gabime/spdlog): 高性能日志库
- [**fmt**](https://github.com/fmtlib/fmt): 现代C++格式化库
- [**Google Test**](https://github.com/google/googletest): 单元测试框架 (可选)

## 🛠️ 编译安装 (cpp)

### 1. 克隆仓库

```bash
git clone https://github.com/Pico-Developer/SpatialMP4
cd SpatialMP4
```

### 2. 编译FFmpeg

首先编译`ffmpeg`：

```bash
bash scripts/build_ffmpeg.sh
```

### 3. 安装依赖

```bash
bash scripts/install_deps.sh
```

### 4. 配置和编译

```bash
mkdir build && cd build

# 配置项目
cmake .. -DCMAKE_BUILD_TYPE=Release

# 编译
make -j$(nproc)  # Linux系统
make -j$(sysctl -n hw.ncpu)  # macOS系统
```
如果编译报错，请参考[FAQ for installation](docs/install_faq.md)。如果还是解决不了，
欢迎[提issue](https://github.com/Pico-Developer/SpatialMP4/issues)，我们会及时帮助你。

### 5. 运行测试 (可选)

```bash
# 编译测试
cmake .. -DBUILD_TESTING=ON
make -j$(nproc)  # Linux系统
make -j$(sysctl -n hw.ncpu)  # macOS系统

# 运行测试
cd ..
./build/test_reader
```

## 🛠️ 编译安装 (python)

### 1. 克隆代码

```bash
git clone https://github.com/Pico-Developer/SpatialMP4
cd SpatialMP4
```

### 2. 编译FFmpeg

Build `ffmpeg` first:

```bash
bash scripts/build_ffmpeg.sh
```

### 3. 安装依赖

```bash
bash scripts/install_deps.sh
```

### 4. 构建安装

```bash
pip3 install .
```

## 📖 使用指南

### 基本用法

```cpp
#include "spatialmp4/reader.h"
#include "spatialmp4/data_types.h"

// 创建读取器
SpatialML::Reader reader("path/to/your/spatial.mp4");

// 检查数据类型
if (reader.HasRGB()) {
    std::cout << "包含RGB数据" << std::endl;
}
if (reader.HasDepth()) {
    std::cout << "包含深度数据" << std::endl;
}
if (reader.HasPose()) {
    std::cout << "包含位姿数据" << std::endl;
}

// 获取相机参数
auto rgb_intrinsics = reader.GetRgbIntrinsicsLeft();
auto depth_intrinsics = reader.GetDepthIntrinsics();
```

### 读取RGB和深度数据

```cpp
// 设置读取模式
reader.SetReadMode(SpatialML::Reader::ReadMode::DEPTH_FIRST);
reader.Reset();

while (reader.HasNext()) {
    SpatialML::rgb_frame rgb_frame;
    SpatialML::depth_frame depth_frame;
    
    // 同时读取RGB和深度帧
    reader.Load(rgb_frame, depth_frame);
    
    // 处理数据
    cv::Mat left_rgb = rgb_frame.left_rgb;
    cv::Mat right_rgb = rgb_frame.right_rgb;
    cv::Mat depth = depth_frame.depth;
    
    std::cout << "RGB时间戳: " << rgb_frame.timestamp << std::endl;
    std::cout << "深度时间戳: " << depth_frame.timestamp << std::endl;
}
```

### 仅读取RGB数据

```cpp
reader.SetReadMode(SpatialML::Reader::ReadMode::RGB_ONLY);
reader.Reset();

while (reader.HasNext()) {
    SpatialML::rgb_frame rgb_frame;
    reader.Load(rgb_frame);
    
    // 处理RGB数据
    cv::imshow("Left RGB", rgb_frame.left_rgb);
    cv::imshow("Right RGB", rgb_frame.right_rgb);
    cv::waitKey(1);
}
```

### 位姿数据处理

```cpp
// 获取所有位姿数据
auto pose_frames = reader.GetPoseFrames();

for (const auto& pose : pose_frames) {
    // 转换为SE(3)表示
    Sophus::SE3d se3_pose = pose.as_se3();
    
    // 获取旋转和平移
    Eigen::Matrix3d rotation = se3_pose.rotationMatrix();
    Eigen::Vector3d translation = se3_pose.translation();
    
    std::cout << "位姿时间戳: " << pose.timestamp << std::endl;
    std::cout << "位置: " << translation.transpose() << std::endl;
}
```

### 深度投影到RGB

```cpp
#include "utilities/RgbdUtils.h"

// 获取相机参数
auto rgb_intrinsics = reader.GetRgbIntrinsicsLeft().as_cvmat();
auto depth_intrinsics = reader.GetDepthIntrinsics().as_cvmat();

// 计算变换矩阵
auto T_I_Srgb = reader.GetRgbExtrinsicsLeft().as_se3();
auto T_I_Stof = reader.GetDepthExtrinsics().as_se3();
auto T_Srgb_Stof = T_I_Srgb.inverse() * T_I_Stof;

// 投影深度到RGB
cv::Mat projected_depth;
Utilities::ProjectDepthToRgb(depth_frame.depth, rgb_frame.left_rgb, 
                           rgb_intrinsics, depth_intrinsics, 
                           T_Srgb_Stof, projected_depth);
```

### 生成点云

```cpp
#include "utilities/PointcloudUtils.h"

// 从RGBD数据生成点云
Utilities::Pointcloud pcd;
Utilities::RgbdToPointcloud(rgb_frame.left_rgb, projected_depth, 
                          rgb_intrinsics, pcd, 10.0f);

// 保存点云
Utilities::SavePointcloudToFile("output.obj", pcd);
```

## 📚 API 参考

### SpatialML::Reader

主要的SpatialMP4文件读取器类。

#### 构造函数
```cpp
Reader(const std::string& filename)
```

#### 数据检查方法
```cpp
bool HasRGB() const;        // 是否包含RGB数据
bool HasDepth() const;      // 是否包含深度数据  
bool HasPose() const;       // 是否包含位姿数据
bool HasAudio() const;      // 是否包含音频数据
bool HasDisparity() const;  // 是否包含视差数据
```

#### 参数获取方法
```cpp
camera_intrinsics GetRgbIntrinsicsLeft() const;   // 左RGB相机内参
camera_intrinsics GetRgbIntrinsicsRight() const;  // 右RGB相机内参
camera_extrinsics GetRgbExtrinsicsLeft() const;   // 左RGB相机外参
camera_extrinsics GetRgbExtrinsicsRight() const;  // 右RGB相机外参
camera_intrinsics GetDepthIntrinsics() const;     // 深度相机内参
camera_extrinsics GetDepthExtrinsics() const;     // 深度相机外参
```

#### 读取控制方法
```cpp
void SetReadMode(ReadMode mode);  // 设置读取模式
bool HasNext() const;             // 是否有下一帧
void Reset();                     // 重置到开始
int GetIndex() const;             // 获取当前索引
```

#### 数据加载方法
```cpp
void Load(rgb_frame& rgb_frame);                              // 加载RGB帧
void Load(depth_frame& depth_frame);                          // 加载深度帧  
void Load(rgb_frame& rgb_frame, depth_frame& depth_frame);    // 同时加载RGB和深度帧
```

### 数据结构

#### rgb_frame
```cpp
struct rgb_frame {
    double timestamp;     // 时间戳
    cv::Mat left_rgb;     // 左眼RGB图像
    cv::Mat right_rgb;    // 右眼RGB图像
    pose_frame pose;      // 对应的位姿数据
};
```

#### depth_frame
```cpp
struct depth_frame {
    double timestamp;     // 时间戳
    cv::Mat depth;        // 深度图像
    pose_frame pose;      // 对应的位姿数据
};
```

#### pose_frame
```cpp
struct pose_frame {
    double timestamp;     // 时间戳
    double x, y, z;       // 位置
    double qw, qx, qy, qz; // 四元数旋转
    
    Sophus::SE3d as_se3() const;  // 转换为SE(3)表示
};
```

## 📖 使用指南 (python)

### 基本用法示例

```python
import spatialmp4

# 创建读取器
reader = spatialmp4.Reader("your_video.mp4")

# 检查可用流
print("Has RGB:", reader.has_rgb())
print("Has Depth:", reader.has_depth())
print("Has Pose:", reader.has_pose())

# 设置读取模式
reader.set_read_mode(spatialmp4.ReadMode.DEPTH_FIRST)

# 读取帧
while reader.has_next():
    rgb_frame, depth_frame = reader.load_both()
    left_rgb = rgb_frame.left_rgb  # numpy数组 (H, W, 3)
    depth = depth_frame.depth      # numpy数组 (H, W)
    pose = rgb_frame.pose
    print("RGB时间戳:", rgb_frame.timestamp, "位姿:", pose.x, pose.y, pose.z)
```

---

## 📚 API 参考 (python)

### 主要类与方法

#### `spatialmp4.Reader`
SpatialMP4 文件读取主类。

- `Reader(filename: str)` — 创建读取器。
- `has_rgb() -> bool` — 是否包含RGB数据。
- `has_depth() -> bool` — 是否包含深度数据。
- `has_pose() -> bool` — 是否包含位姿数据。
- `has_audio() -> bool` — 是否包含音频数据。
- `has_disparity() -> bool` — 是否包含视差数据。
- `get_duration() -> float` — 获取视频时长（秒）。
- `get_rgb_fps() -> float` — 获取RGB帧率。
- `get_depth_fps() -> float` — 获取深度帧率。
- `get_rgb_width() -> int` — 获取RGB宽度。
- `get_rgb_height() -> int` — 获取RGB高度。
- `get_depth_width() -> int` — 获取深度宽度。
- `get_depth_height() -> int` — 获取深度高度。
- `get_rgb_intrinsics_left() -> CameraIntrinsics` — 获取左RGB相机内参。
- `get_rgb_intrinsics_right() -> CameraIntrinsics` — 获取右RGB相机内参。
- `get_rgb_extrinsics_left() -> CameraExtrinsics` — 获取左RGB相机外参。
- `get_rgb_extrinsics_right() -> CameraExtrinsics` — 获取右RGB相机外参。
- `get_depth_intrinsics() -> CameraIntrinsics` — 获取深度相机内参。
- `get_depth_extrinsics() -> CameraExtrinsics` — 获取深度相机外参。
- `get_pose_frames() -> List[PoseFrame]` — 获取所有位姿帧。
- `set_read_mode(mode: ReadMode)` — 设置读取模式（见下方枚举）。
- `has_next() -> bool` — 是否有下一帧。
- `reset()` — 重置到文件开头。
- `get_index() -> int` — 获取当前帧索引。
- `get_frame_count() -> int` — 获取总帧数。
- `load_rgb() -> RGBFrame` — 读取下一个RGB帧。
- `load_depth(raw_head_pose: bool = False) -> DepthFrame` — 读取下一个深度帧。
- `load_both() -> (RGBFrame, DepthFrame)` — 同时读取下一个RGB和深度帧。
- `load_rgbd(densify: bool = False) -> Rgbd` — 读取RGBD数据（高级用法）。

#### `spatialmp4.RGBFrame`
- `timestamp: float` — 帧时间戳。
- `left_rgb: np.ndarray` — 左RGB图像 (H, W, 3, uint8)。
- `right_rgb: np.ndarray` — 右RGB图像 (H, W, 3, uint8)。
- `pose: PoseFrame` — 对应位姿数据。

#### `spatialmp4.DepthFrame`
- `timestamp: float` — 帧时间戳。
- `depth: np.ndarray` — 深度图像 (H, W, float32, 单位米)。
- `pose: PoseFrame` — 对应位姿数据。

#### `spatialmp4.PoseFrame`
- `timestamp: float` — 位姿时间戳。
- `x, y, z: float` — 位置。
- `qw, qx, qy, qz: float` — 四元数旋转。
- `as_se3()` — 转换为SE(3)表示（需Sophus/Eigen，高级用法）。

#### `spatialmp4.CameraIntrinsics`
- `fx, fy, cx, cy: float` — 相机内参。
- `as_cvmat()` — 以OpenCV矩阵返回。

#### `spatialmp4.CameraExtrinsics`
- `extrinsics: np.ndarray` — 4x4外参矩阵。
- `as_cvmat()` — 以OpenCV矩阵返回。
- `as_se3()` — 以SE(3)返回（高级用法）。

---

### 枚举类型

#### `spatialmp4.ReadMode`
- `RGB_ONLY` — 仅读取RGB帧。
- `DEPTH_ONLY` — 仅读取深度帧。
- `DEPTH_FIRST` — 同时读取RGB和深度帧，以深度为参考。

#### `spatialmp4.StreamType`
- `UNKNOWN` — 未知流类型
- `AUDIO` — 音频流
- `AUDIO_2` — 第二音频流
- `RGB` — RGB视频流
- `DISPARITY` — 视差流
- `POSE` — 位姿数据流
- `DEPTH` — 深度流


### 高级用法

- 参见 [examples/python/visualize_rerun.py](./examples/python/visualize_rerun.py) 和 [examples/python/generate_pcd.py](./examples/python/generate_pcd.py) 获取点云生成、Open3D/Rerun可视化等高级用法。
- 所有图像和深度数据均以NumPy数组返回，便于与OpenCV、Open3D、PyTorch等生态集成。
- 相机参数和位姿数据可用于三维重建和SLAM等应用。


## 🐛 调试和日志

项目使用spdlog进行日志记录：

```cpp
#include <spdlog/spdlog.h>

// 设置日志级别
spdlog::set_level(spdlog::level::debug);

// 在代码中会自动输出调试信息
```

## 📄 许可证

本项目采用MIT许可证。详见 [LICENSE](LICENSE) 文件。

## 🤝 贡献

欢迎提交Issue和Pull Request来改进这个项目！

## 📞 联系方式

如有问题或建议，请通过GitHub Issues联系我们。
