# Usage:
# >> include(cmake/sophus.cmake)
# >> "Then you can add `Sophus` as link library to your target."
#

include(FetchContent)
FetchContent_Declare(
    sophus
    GIT_REPOSITORY https://github.com/strasdat/Sophus.git
    GIT_TAG 1.22.10
    GIT_SHALLOW TRUE
    GIT_PROGRESS TRUE
)

# Set Sophus build options before making it available
set(BUILD_SOPHUS_TESTS OFF CACHE BOOL "")
set(BUILD_SOPHUS_EXAMPLES OFF CACHE BOOL "")

FetchContent_MakeAvailable(sophus)

# Ensure Sophus uses C++14 or higher
if(TARGET sophus)
    set_target_properties(sophus PROPERTIES
        CXX_STANDARD 14
        CXX_STANDARD_REQUIRED ON
        CXX_EXTENSIONS OFF
    )
endif()

include_directories(${sophus_SOURCE_DIR})
message(STATUS "SOPHUS_INCLUDE_DIR: ${sophus_SOURCE_DIR}")