# Usage:
#   >> include(../cmake/ffmpeg_local.cmake)
#   >> target_link_libraries( ... PkgConfig::LIBAV)
# 
# Install library by `scripts/install/install_ffmpeg.sh`

# 设置 PKG_CONFIG_PATH 环境变量
set(LIBAV_USE_STATIC_PKG_CONFIG OFF)
if(DEFINED ENV{CONDA_PREFIX})
    set(ffmpeg_HOME $ENV{CONDA_PREFIX})
else()
    set(ffmpeg_HOME ${CMAKE_CURRENT_SOURCE_DIR}/scripts/build_ffmpeg/ffmpeg_install)
    if(EXISTS "${ffmpeg_HOME}/lib/pkgconfig")
        set(LIBAV_USE_STATIC_PKG_CONFIG ON)
    endif()
endif()

if(EXISTS "${ffmpeg_HOME}/lib/pkgconfig")
    set(ENV{PKG_CONFIG_PATH} "${ffmpeg_HOME}/lib/pkgconfig:$ENV{PKG_CONFIG_PATH}")
endif()

if(LIBAV_USE_STATIC_PKG_CONFIG)
    set(ENV{PKG_CONFIG} "pkg-config --static")
endif()

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

if(APPLE AND TARGET PkgConfig::LIBAV)
    # FindPkgConfig exposes static pkg-config framework flags as
    # "-framework;Foo;-framework;Bar" in INTERFACE_LINK_OPTIONS. CMake's
    # option de-duplication can collapse the repeated "-framework" tokens and
    # leave bare framework names on the compiler command line. Group each pair
    # with SHELL: so clang receives valid "-framework Foo" arguments.
    get_target_property(LIBAV_LINK_OPTIONS PkgConfig::LIBAV INTERFACE_LINK_OPTIONS)
    if(LIBAV_LINK_OPTIONS)
        set(LIBAV_FIXED_LINK_OPTIONS "")
        set(LIBAV_EXPECT_FRAMEWORK_NAME OFF)
        foreach(LIBAV_LINK_OPTION IN LISTS LIBAV_LINK_OPTIONS)
            if(LIBAV_EXPECT_FRAMEWORK_NAME)
                list(APPEND LIBAV_FIXED_LINK_OPTIONS "SHELL:-framework ${LIBAV_LINK_OPTION}")
                set(LIBAV_EXPECT_FRAMEWORK_NAME OFF)
            elseif(LIBAV_LINK_OPTION STREQUAL "-framework")
                set(LIBAV_EXPECT_FRAMEWORK_NAME ON)
            else()
                list(APPEND LIBAV_FIXED_LINK_OPTIONS "${LIBAV_LINK_OPTION}")
            endif()
        endforeach()
        set_target_properties(PkgConfig::LIBAV PROPERTIES
            INTERFACE_LINK_OPTIONS "${LIBAV_FIXED_LINK_OPTIONS}"
        )
    endif()
endif()

message(STATUS "ffmpeg include path: ${LIBAV_INCLUDE_DIRS}")
message(STATUS "ffmpeg lib path: ${LIBAV_LIBRARIES}")
