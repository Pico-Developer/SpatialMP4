# Usage:
#   >> include(../cmake/ffmpeg_local.cmake)
#   >> target_link_libraries( ... PkgConfig::LIBAV)
# 
# Install library by `scripts/install/install_ffmpeg.sh`

# 设置 PKG_CONFIG_PATH 环境变量
set(ffmpeg_HOME ${CMAKE_CURRENT_SOURCE_DIR}/scripts/build_ffmpeg/ffmpeg_install)
set(ENV{PKG_CONFIG_PATH} "${ffmpeg_HOME}/lib/pkgconfig:$ENV{PKG_CONFIG_PATH}")

find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBAV REQUIRED IMPORTED_TARGET
    libavdevice
    libavfilter
    libavformat
    libavcodec
    libavutil
    libswresample
    libswscale
)

message(STATUS "ffmpeg include path: ${LIBAV_INCLUDE_DIRS}")
message(STATUS "ffmpeg lib path: ${LIBAV_LIBRARIES}")