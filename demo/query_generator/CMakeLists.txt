project(query_generator)

add_executable(query_generator main_query_generator.cpp)
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_CURRENT_SOURCE_DIR}/../../bin/query_generator)

target_link_libraries(query_generator
    ${OpenCV_LIBS}
	)