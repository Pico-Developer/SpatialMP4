include(FetchContent)
FetchContent_Declare(
    eigen
    URL https://gitlab.com/libeigen/eigen/-/archive/3.3.7/eigen-3.3.7.tar.gz
    DOWNLOAD_EXTRACT_TIMESTAMP true
    )
FetchContent_MakeAvailable(eigen)
include_directories(${eigen_SOURCE_DIR})
message(STATUS "EIGEN_INCLUDE_DIR: ${eigen_SOURCE_DIR}")
