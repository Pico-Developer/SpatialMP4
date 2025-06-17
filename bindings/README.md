# SpatialMP4 Python Bindings

This directory contains Python bindings for the SpatialMP4 library using pybind11.

## Requirements

- Python 3.6 or later
- CMake 3.24 or later
- pybind11
- OpenCV Python
- NumPy

## Building the Bindings

1. Make sure you have all the dependencies installed:

```bash
# Install Python dependencies
pip install numpy opencv-python

# Install pybind11 (Ubuntu/Debian)
sudo apt-get install pybind11-dev

# Install pybind11 (macOS)
brew install pybind11
```

2. Build the project with Python bindings enabled:

```bash
mkdir build && cd build
cmake .. -DBUILD_PYTHON_BINDINGS=ON
make -j
```

The Python module will be built in the `build/python` directory.

## Installation

After building, you can install the Python module system-wide:

```bash
sudo make install
```

Or for development, you can add the build directory to your PYTHONPATH:

```bash
export PYTHONPATH=/path/to/build/python:$PYTHONPATH
```

## Usage

See `example.py` for a complete example of how to use the Python bindings. Here's a basic example:

```python
import spatialmp4

# Create a reader
reader = spatialmp4.Reader("path/to/your/video.mp4")

# Check available streams
print(f"Has RGB: {reader.has_rgb()}")
print(f"Has Depth: {reader.has_depth()}")
print(f"Has Pose: {reader.has_pose()}")

# Set read mode
reader.set_read_mode(spatialmp4.ReadMode.RGB_ONLY)

# Read frames
while reader.has_next():
    rgb_frame = reader.load_rgb()
    left_rgb = rgb_frame.left_rgb  # NumPy array
    right_rgb = rgb_frame.right_rgb  # NumPy array
    pose = rgb_frame.pose
```

## API Reference

### Classes

#### Reader
- Main class for reading SpatialMP4 files
- Methods:
  - `has_rgb()`: Check if RGB stream is available
  - `has_depth()`: Check if depth stream is available
  - `has_pose()`: Check if pose data is available
  - `get_duration()`: Get video duration in seconds
  - `get_rgb_fps()`: Get RGB stream FPS
  - `get_depth_fps()`: Get depth stream FPS
  - And more...

#### RGBFrame
- Contains RGB frame data and associated pose
- Properties:
  - `timestamp`: Frame timestamp
  - `left_rgb`: Left RGB image as NumPy array
  - `right_rgb`: Right RGB image as NumPy array
  - `pose`: Associated pose data

#### DepthFrame
- Contains depth frame data and associated pose
- Properties:
  - `timestamp`: Frame timestamp
  - `depth`: Depth image as NumPy array
  - `pose`: Associated pose data

#### PoseFrame
- Contains pose data
- Properties:
  - `timestamp`: Pose timestamp
  - `x`, `y`, `z`: Position
  - `qw`, `qx`, `qy`, `qz`: Quaternion orientation

### Enums

#### ReadMode
- `RGB_ONLY`: Only read RGB frames
- `DEPTH_ONLY`: Only read depth frames
- `DEPTH_FIRST`: Read both RGB and depth frames, depth frame as reference

#### StreamType
- `UNKNOWN`: Unknown stream type
- `AUDIO`: Audio stream
- `AUDIO_2`: Secondary audio stream
- `RGB`: RGB video stream
- `DISPARITY`: Disparity stream
- `POSE`: Pose data stream
- `DEPTH`: Depth stream 
