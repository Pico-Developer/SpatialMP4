cmake_minimum_required(VERSION 3.24.1 FATAL_ERROR)

project(spatialmp4)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_POLICY_DEFAULT_CMP0077 NEW)
option(BUILD_ANDROID "whether build for android" OFF)
option(BUILD_TESTING "whether build unit test" ON)
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
 
set(SRC 
    ./src/spatialmp4/utilities/OpencvUtils.cc
    ./src/spatialmp4/utils.cc
    ./src/spatialmp4/reader.cc
)

add_library(${CMAKE_PROJECT_NAME}_static STATIC ${SRC})
target_include_directories(${CMAKE_PROJECT_NAME}_static PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)
target_link_libraries(${CMAKE_PROJECT_NAME}_static PUBLIC
    PkgConfig::LIBAV
    lib_opencv
    spdlog
    fmt::fmt
    Eigen3::Eigen
    sophus
)

if(BUILD_TESTING)
    include(cmake/gtest.cmake)
    add_executable(test_reader
        ${SRC}
        ./src/spatialmp4/utilities/RgbdUtils.cc
        ./src/spatialmp4/utilities/PointcloudUtils.cc
        ./src/spatialmp4/reader_test.cc
    )
    target_link_libraries(test_reader PRIVATE
        ${CMAKE_PROJECT_NAME}_static
        gtest
        gtest_main
    )
    
    # On newer versions of macOS (>=10.15) and with newer compilers,
    # filesystem is part of the standard library and doesn't need explicit linking
    if(UNIX AND NOT APPLE)
        target_link_libraries(test_reader PRIVATE stdc++fs)
    endif()
endif()
