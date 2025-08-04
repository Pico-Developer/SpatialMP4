# Usage:
# >> include(cmake/opencv_host.cmake)
# >> "Then you can add `PkgConfig::OpenCV` as link library name to your target."

if(DEFINED ENV{CONDA_PREFIX})
    set(ENV{PKG_CONFIG_PATH} "${CONDA_PREFIX}/lib/pkgconfig:$ENV{PKG_CONFIG_PATH}")
endif()

set(ENV{PKG_CONFIG} "pkg-config --static")

find_package(PkgConfig)
pkg_check_modules(OpenCV REQUIRED IMPORTED_TARGET opencv4)

message(STATUS "OpenCV_INCLUDE_DIRS=${OpenCV_INCLUDE_DIRS}")
message(STATUS "OpenCV_LIBRARIES=${OpenCV_LIBRARIES}")