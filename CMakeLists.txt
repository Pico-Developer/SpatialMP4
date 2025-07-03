cmake_minimum_required(VERSION 3.24.1 FATAL_ERROR)

project(spatialmp4 VERSION 0.1.0)

# Configure version header
configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/src/spatialmp4/version.h.in
    ${CMAKE_CURRENT_BINARY_DIR}/include/spatialmp4/version.h
    @ONLY
)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_SHARED_LINKER_FLAGS "-Wl,-Bsymbolic-functions")

set(CMAKE_POLICY_DEFAULT_CMP0077 NEW)
option(BUILD_ANDROID "whether build for android" OFF)
option(BUILD_TESTING "whether build unit test" ON)
option(BUILD_PYTHON "whether build python bindings" ON)

if(BUILD_ANDROID)
    message(FATAL_ERROR "Don't support android building.")
endif()

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fvisibility=hidden -fPIC")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fvisibility=hidden -fPIC -DEIGEN_MPL2_ONLY")

include(cmake/ffmpeg_local.cmake)
include(cmake/opencv_host.cmake)
include(cmake/spdlog.cmake)
include(cmake/fmt.cmake)
include(cmake/eigen.cmake)
include(cmake/sophus.cmake)

if(BUILD_PYTHON)
    include(cmake/pybind11.cmake)
endif()
 
set(SRC 
    ./src/spatialmp4/utilities/OpencvUtils.cc
    ./src/spatialmp4/utils.cc
    ./src/spatialmp4/reader.cc
)

add_library(${CMAKE_PROJECT_NAME}_lib STATIC ${SRC})
# if(APPLE)
#     add_library(${CMAKE_PROJECT_NAME}_lib STATIC ${SRC})
# else()
#     add_library(${CMAKE_PROJECT_NAME}_lib SHARED ${SRC})
# endif()

target_include_directories(${CMAKE_PROJECT_NAME}_lib PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMAKE_CURRENT_BINARY_DIR}/include
)


if(EXISTS $ENV{CONDA_PREFIX}/lib)
    link_directories($ENV{CONDA_PREFIX}/lib)
endif()

target_link_libraries(${CMAKE_PROJECT_NAME}_lib PUBLIC
    PkgConfig::LIBAV
    PkgConfig::OpenCV
    spdlog
    fmt::fmt
    Eigen3::Eigen
    sophus
)
# On newer versions of macOS (>=10.15) and with newer compilers,
# filesystem is part of the standard library and doesn't need explicit linking
if(UNIX AND NOT APPLE)
    target_link_libraries(${CMAKE_PROJECT_NAME}_lib PRIVATE stdc++fs)
endif()


# Add macOS framework linking
if(APPLE)
    set_target_properties(${CMAKE_PROJECT_NAME}_lib PROPERTIES
        INSTALL_RPATH "@loader_path"
        BUILD_WITH_INSTALL_RPATH TRUE
        LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/python
    )
    
    # Find macOS frameworks
    find_library(FOUNDATION_FRAMEWORK Foundation)
    find_library(AUDIOTOOLBOX_FRAMEWORK AudioToolbox)
    find_library(COREAUDIO_FRAMEWORK CoreAudio)
    find_library(AVFOUNDATION_FRAMEWORK AVFoundation)
    find_library(COREVIDEO_FRAMEWORK CoreVideo)
    find_library(COREMEDIA_FRAMEWORK CoreMedia)
    find_library(COREGRAPHICS_FRAMEWORK CoreGraphics)
    find_library(OPENGL_FRAMEWORK OpenGL)
    find_library(APPLICATIONSERVICES_FRAMEWORK ApplicationServices)
    find_library(COREFOUNDATION_FRAMEWORK CoreFoundation)
    find_library(APPKIT_FRAMEWORK AppKit)
    find_library(CARBON_FRAMEWORK Carbon)
    find_library(METAL_FRAMEWORK Metal)
    find_library(VIDEOTOOLBOX_FRAMEWORK VideoToolbox)
    find_library(COREIMAGE_FRAMEWORK CoreImage)
    find_library(CORESERVICES_FRAMEWORK CoreServices)
    find_library(SECURITY_FRAMEWORK Security)
    
    target_link_libraries(${CMAKE_PROJECT_NAME}_lib PUBLIC
        ${FOUNDATION_FRAMEWORK}
        ${AUDIOTOOLBOX_FRAMEWORK}
        ${COREAUDIO_FRAMEWORK}
        ${AVFOUNDATION_FRAMEWORK}
        ${COREVIDEO_FRAMEWORK}
        ${COREMEDIA_FRAMEWORK}
        ${COREGRAPHICS_FRAMEWORK}
        ${OPENGL_FRAMEWORK}
        ${APPLICATIONSERVICES_FRAMEWORK}
        ${COREFOUNDATION_FRAMEWORK}
        ${APPKIT_FRAMEWORK}
        ${CARBON_FRAMEWORK}
        ${METAL_FRAMEWORK}
        ${VIDEOTOOLBOX_FRAMEWORK}
        ${COREIMAGE_FRAMEWORK}
        ${CORESERVICES_FRAMEWORK}
        ${SECURITY_FRAMEWORK}
    )
endif()

if(BUILD_TESTING)
    include(cmake/gtest.cmake)
    add_executable(test_reader
        ${SRC}
        ./src/spatialmp4/utilities/RgbdUtils.cc
        ./src/spatialmp4/utilities/PointcloudUtils.cc
        ./src/spatialmp4/reader_test.cc
    )
    target_link_libraries(test_reader PRIVATE
        ${CMAKE_PROJECT_NAME}_lib
        gtest
        gtest_main
    )
    
    # On newer versions of macOS (>=10.15) and with newer compilers,
    # filesystem is part of the standard library and doesn't need explicit linking
    if(UNIX AND NOT APPLE)
        target_link_libraries(test_reader PRIVATE stdc++fs)
    endif()
endif()

if(BUILD_PYTHON)
    add_subdirectory(bindings)
endif()

if (APPLE)
    set_target_properties(${CMAKE_PROJECT_NAME}_lib PROPERTIES INSTALL_RPATH "@loader_path")
else()
    set_target_properties(${CMAKE_PROJECT_NAME}_lib PROPERTIES INSTALL_RPATH "\$ORIGIN")
endif()
install(TARGETS ${CMAKE_PROJECT_NAME}_lib DESTINATION lib)
