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
FetchContent_MakeAvailable(sophus)
include_directories(${sophus_SOURCE_DIR})
message(STATUS "SOPHUS_INCLUDE_DIR: ${sophus_SOURCE_DIR}")