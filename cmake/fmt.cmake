#
# Add in your CMakeLists.txt
# >> include("cmake/fmt.cmake")
# >> target_link_libraries
#       ...
#       fmt::fmt
#    )
include(FetchContent)
FetchContent_Declare(
    fmt
    GIT_REPOSITORY https://github.com/fmtlib/fmt.git
    GIT_TAG 9.1.0  # 指定版本，可以更改为最新版本
)
FetchContent_MakeAvailable(fmt)
include_directories(${fmt_SOURCE_DIR})
