# Usage:
# >> include(cmake/opencv_host.cmake)
# >> "Then you can add `lib_opencv` as link library name to your target."

find_package( OpenCV REQUIRED )
add_compile_options(-std=c++11)

# find_package(PkgConfig)
# pkg_check_modules(OpenCV REQUIRED opencv>=2.4.9)
# set(OpenCV_INCLUDE_DIRS ${OpenCV_INCLUDE_DIRS} /usr/local/opencv_itseez/include/opencv2)
# set(OpenCV_LIBS ${OpenCV_LIBRARIES})
# LINK_DIRECTORIES(${OpenCV_LIBDIR}) 

include_directories(${OpenCV_INCLUDE_DIRS})
add_library(lib_opencv INTERFACE)
target_link_libraries(lib_opencv INTERFACE ${OpenCV_LIBS})

message(STATUS "OpenCV_INCLUDE_DIRS=${OpenCV_INCLUDE_DIRS}")
message(STATUS "OpenCV_LIBS=${OpenCV_LIBS}")
