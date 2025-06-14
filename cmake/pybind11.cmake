include(FetchContent)

# 首先查找 Python
find_package(Python3 COMPONENTS Interpreter Development REQUIRED)

# 设置 Python 相关变量
set(PYTHON_EXECUTABLE ${Python3_EXECUTABLE})
set(PYTHON_INCLUDE_DIR ${Python3_INCLUDE_DIRS})
set(PYTHON_LIBRARY ${Python3_LIBRARIES})

# 配置 pybind11
FetchContent_Declare(
    pybind11
    GIT_REPOSITORY https://github.com/pybind/pybind11.git
    GIT_TAG        v2.11.1  # 使用最新的稳定版本
)

# 设置 pybind11 选项
set(PYBIND11_PYTHON_VERSION ${Python3_VERSION})
set(PYBIND11_FINDPYTHON ON)
set(PYBIND11_NOPYTHON OFF)

# 获取并配置 pybind11
FetchContent_MakeAvailable(pybind11)

# 确保 pybind11 被正确配置
if(NOT TARGET pybind11::module)
    message(FATAL_ERROR "pybind11::module target not found")
endif() 