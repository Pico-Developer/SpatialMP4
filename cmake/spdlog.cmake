# Usage:
# >> include(cmake/spdlog.cmake)
# >> "Then you can add `spdlog` as link library to your target."
#
# If you want to use it on android, add below in main entry:
#
#  include <spdlog/spdlog.h>
#  include <spdlog/sinks/android_sink.h>

#    std::shared_ptr<spdlog::logger> android_logger = spdlog::android_logger_mt("android", "TAG");
#    spdlog::set_pattern("%v");
#    spdlog::set_default_logger(android_logger);
# 
# Usage:
# #include <spdlog/spdlog>
# std::string who = "world";
# spdlog::info("Hello, {}", world);
# spdlog::debug("Hello, {}", world);
#

include(FetchContent)
FetchContent_Declare(spdlog
    GIT_REPOSITORY https://github.com/gabime/spdlog.git
    GIT_TAG v1.13.0
    GIT_SHALLOW TRUE
    GIT_PROGRESS TRUE
)
FetchContent_MakeAvailable(spdlog)
