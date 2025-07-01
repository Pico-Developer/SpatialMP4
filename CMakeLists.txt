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

set(CMAKE_POLICY_DEFAULT_CMP0077 NEW)
option(BUILD_ANDROID "whether build for android" OFF)
option(BUILD_TESTING "whether build unit test" ON)
option(BUILD_PYTHON_BINDINGS "whether build python bindings" ON)

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

if(BUILD_PYTHON_BINDINGS)
    include(cmake/pybind11.cmake)
endif()
 
set(SRC 
    ./src/spatialmp4/utilities/OpencvUtils.cc
    ./src/spatialmp4/utils.cc
    ./src/spatialmp4/reader.cc
)

add_library(${CMAKE_PROJECT_NAME}_lib SHARED ${SRC})
target_include_directories(${CMAKE_PROJECT_NAME}_lib PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMAKE_CURRENT_BINARY_DIR}/include
)

target_link_libraries(${CMAKE_PROJECT_NAME}_lib PUBLIC
    PkgConfig::LIBAV
    lib_opencv
    spdlog
    fmt::fmt
    Eigen3::Eigen
    sophus
)
# On newer versions of macOS (>=10.15) and with newer compilers,
# filesystem is part of the standard library and doesn't need explicit linking
if(UNIX AND NOT APPLE)
    target_link_libraries(${CMAKE_PROJECT_NAME}_static PRIVATE stdc++fs)
endif()


# Add macOS framework linking
if(APPLE)
    set_target_properties(${CMAKE_PROJECT_NAME}_lib PROPERTIES
        INSTALL_RPATH "@loader_path"
        BUILD_WITH_INSTALL_RPATH TRUE
        LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/python
    )
    target_link_libraries(${CMAKE_PROJECT_NAME}_lib PUBLIC
        "-framework Foundation"
        "-framework AudioToolbox"
        "-framework CoreAudio"
        "-framework AVFoundation"
        "-framework CoreVideo"
        "-framework CoreMedia"
        "-framework CoreGraphics"
        "-framework OpenGL"
        "-framework ApplicationServices"
        "-framework CoreFoundation"
        "-framework AppKit"
        "-framework Carbon"
        "-framework Metal"
        "-framework VideoToolbox"
        "-framework CoreImage"
        "-framework CoreServices"
        "-framework Security"
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

if(BUILD_PYTHON_BINDINGS)
    add_subdirectory(bindings)
endif()

if (APPLE)
    set_target_properties(${CMAKE_PROJECT_NAME}_lib PROPERTIES INSTALL_RPATH "@loader_path")
else()
    set_target_properties(${CMAKE_PROJECT_NAME}_lib PROPERTIES INSTALL_RPATH "\$ORIGIN")
endif()
install(TARGETS ${CMAKE_PROJECT_NAME}_lib DESTINATION lib)
