#
# Add in your CMakeLists.txt
# >> include("cmake/gtest.cmake")
# >> target_link_libraries
#       ...
#       gtest gtest_main
#    )
include(FetchContent)

FetchContent_Declare(
    gtest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG        release-1.11.0  # You can specify the version you need
)

FetchContent_MakeAvailable(gtest)

set(gtest_DIR ${gtest_SOURCE_DIR})
message(STATUS "gtest_DIR=${gtest_DIR}")

include_directories(${gtest_SOURCE_DIR}/googletest/include)