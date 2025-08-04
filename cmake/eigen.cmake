include(FetchContent)
FetchContent_Declare(
    eigen
    # URL https://gitlab.com/libeigen/eigen/-/archive/3.3.7/eigen-3.3.7.tar.gz
    URL https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.tar.gz
    DOWNLOAD_EXTRACT_TIMESTAMP true
)

# Set Eigen build options
set(EIGEN_BUILD_DOC OFF CACHE BOOL "Don't build Eigen docs")
set(BUILD_TESTING OFF CACHE BOOL "Don't build Eigen tests")
set(EIGEN_BUILD_PKGCONFIG OFF CACHE BOOL "Don't build pkg-config files")

FetchContent_MakeAvailable(eigen)

# Set up Eigen3::Eigen target that Ceres expects
if(NOT TARGET Eigen3::Eigen)
    add_library(Eigen3::Eigen INTERFACE IMPORTED)
    set_target_properties(Eigen3::Eigen PROPERTIES
        INTERFACE_INCLUDE_DIRECTORIES "${eigen_SOURCE_DIR}")
endif()

message(STATUS "EIGEN_INCLUDE_DIR: ${eigen_SOURCE_DIR}")
