project(image_generator)

include_directories(
    ../../src/rendering
	)

add_executable(image_generator main_image_generator.cpp)
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_CURRENT_SOURCE_DIR}/../../bin/image_generator)

target_link_libraries(image_generator
    Rendering
	)