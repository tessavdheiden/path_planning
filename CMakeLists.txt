cmake_minimum_required(VERSION 3.8)
project(Bachelor)

add_subdirectory(visualizer)
add_subdirectory(implementation)

set(CMAKE_THREAD_PREFER_PTHREAD TRUE)
set(THREADS_PREFER_PTHREAD_FLAG TRUE)
find_package(Threads REQUIRED)

add_executable(Bachelor main.cpp)
target_link_libraries(Bachelor visualizer)
target_link_libraries(Bachelor implementation)
target_link_libraries(Bachelor Threads::Threads)
add_custom_command(
    TARGET Bachelor
    POST_BUILD COMMAND
        ${CMAKE_COMMAND} -E copy_directory
        "${CMAKE_CURRENT_SOURCE_DIR}/assets"
        $<TARGET_FILE_DIR:Bachelor>/assets)





