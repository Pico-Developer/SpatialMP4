# Usage:
# >> include(cmake/opencv_host.cmake)
# >> "Then you can add `lib_opencv` as link library name to your target."


# set(PIXI_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/.pixi/envs/default)
# if(EXISTS "${PIXI_ROOT}") 
#     set(OpenCV_DIR ${CONDA_PREFIX}/share/opencv4)
# endif()
# set(OpenCV_STATIC ON)
# find_package( OpenCV REQUIRED )
# add_compile_options(-std=c++11)
# include_directories(${OpenCV_INCLUDE_DIRS})
# add_library(lib_opencv INTERFACE)
# target_link_libraries(lib_opencv INTERFACE ${OpenCV_LIBS})
# set(OpenCV_INCLUDE_DIRS ${OpenCV_INCLUDE_DIRS})
# set(OpenCV_LIBS ${OpenCV_LIBRARIES})
# LINK_DIRECTORIES(${OpenCV_LIBDIR})

set(PIXI_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/.pixi/envs/default)
if(EXISTS "${PIXI_ROOT}") 
    set(ENV{PKG_CONFIG_PATH} "${PIXI_ROOT}/lib/pkgconfig:$ENV{PKG_CONFIG_PATH}")
endif()
set(ENV{PKG_CONFIG} "pkg-config --static")

find_package(PkgConfig)
pkg_check_modules(OpenCV REQUIRED IMPORTED_TARGET opencv4)

message(STATUS "OpenCV_INCLUDE_DIRS=${OpenCV_INCLUDE_DIRS}")
message(STATUS "OpenCV_LIBRARIES=${OpenCV_LIBRARIES}")