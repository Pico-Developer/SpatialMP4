# Usage:
#   >> include(../cmake/ffmpeg_local.cmake)
#   >> target_link_libraries( ... PkgConfig::LIBAV)
# 
# Install library by `scripts/install/install_ffmpeg.sh`

# 设置 PKG_CONFIG_PATH 环境变量
set(ENV{PKG_CONFIG_PATH} "$ENV{HOME}/opt/ffmpeg_install/lib/pkgconfig:$ENV{PKG_CONFIG_PATH}")

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