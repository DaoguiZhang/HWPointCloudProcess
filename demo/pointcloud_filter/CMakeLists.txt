project(pointcloud_filter)

include_directories(
    ../../src/segmentation
	)

add_executable(pointcloud_filter main_pointcloud_filter.cpp)
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_CURRENT_SOURCE_DIR}/../../bin/pointcloud_filter)

target_link_libraries(pointcloud_filter Segmentation)