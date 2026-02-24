cmake_minimum_required(VERSION 3.1)
project(generate_marker)

set (CMAKE_CXX_STANDARD 11)
set(OpenCV_DIR "/usr/local/include/opencv4")
find_package(OpenCV REQUIRED)

include_directories(${OPENCV_INCLUDE_DIRS})
include_directories(${PROJECT_SOURCE_DIR}/include)

link_directories(${OpenCV_LIBRARY_DIRS})
set(gererate_marker_src
    src/generate_marker.cpp
   )

add_executable(generate_marker ${gererate_marker_src})
target_link_libraries(generate_marker
    ${OpenCV_LIBRARIES}
    )

set(generate_board_src
    src/generate_board.cpp
   )

add_executable(generate_board ${generate_board_src})
target_link_libraries(generate_board
    ${OpenCV_LIBRARIES}
    )

set(detecting_aruco_src
    src/detecting_aruco.cpp
)

add_executable(detecting_aruco ${detecting_aruco_src})
target_link_libraries(detecting_aruco
    ${OpenCV_LIBRARIES}
    )

target_compile_options(generate_marker
    PRIVATE -O3 -std=c++11
    )


