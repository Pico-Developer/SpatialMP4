cmake_minimum_required(VERSION 3.14)

if (NOT PYTHON_SITELIB)
set(PYTHON_SITELIB "lib")
endif()

if ("${CMAKE_LIBRARY_OUTPUT_DIRECTORY}" STREQUAL "")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR})
endif()

set(CMAKE_SHARED_MODULE_PREFIX "")

# Find Python
find_package(Python3 COMPONENTS Interpreter Development REQUIRED)

# Add Python bindings library
pybind11_add_module(spatialmp4 spatialmp4.cpp)

# Link against main library and dependencies
target_link_libraries(spatialmp4 PRIVATE 
    spatialmp4_lib
    ${OpenCV_LIBS}
    Sophus::Sophus
    Eigen3::Eigen
    ${FFMPEG_LIBRARIES}
)

# Include directories
target_include_directories(spatialmp4 PRIVATE
    ${CMAKE_SOURCE_DIR}/src
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${FFMPEG_INCLUDE_DIRS}
)

if (APPLE)
set_target_properties(spatialmp4 PROPERTIES INSTALL_RPATH "@loader_path/../..")
else()
set_target_properties(spatialmp4 PROPERTIES INSTALL_RPATH "\$ORIGIN/../..")
endif()

# Set output directory
set_target_properties(spatialmp4 PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/python
)

# Install rules
install(TARGETS spatialmp4
    LIBRARY DESTINATION ${PYTHON_SITELIB}
    RUNTIME DESTINATION ${PYTHON_SITELIB}
) 

# add_custom_command(
#     TARGET spatialmp4 POST_BUILD
#     COMMAND doxystub
#         --module spatialmp4
#         --doxygen_directory "${CMAKE_CURRENT_SOURCE_DIR}"
#         --output "${CMAKE_BINARY_DIR}/${PYTHON_SITELIB}/spatialmp4.pyi"
#     WORKING_DIRECTORY "${CMAKE_BINARY_DIR}/${PYTHON_SITELIB}"
#     COMMENT "Generating stubs..."
# )

# install(FILES ${CMAKE_BINARY_DIR}/${PYTHON_SITELIB}/spatialmp4.pyi DESTINATION ${PYTHON_SITELIB})