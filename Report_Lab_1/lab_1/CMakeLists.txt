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

set(camera_calibrate_src
    src/camera_calibration.cpp
)

add_executable(camera_calibrate ${camera_calibrate_src})
target_link_libraries(camera_calibrate
    ${OpenCV_LIBRARIES}
    )

set(pose_estimation_src
    src/pose_estimation.cpp
)

add_executable(pose_estimation ${pose_estimation_src})
target_link_libraries(pose_estimation
    ${OpenCV_LIBRARIES}
    )

set(draw_cube_src
    src/draw_cube.cpp
)

add_executable(draw_cube ${draw_cube_src})
target_link_libraries(draw_cube
    ${OpenCV_LIBRARIES}
    )

set(relative_pose_estimation_src
    src/relative_pose_estimation.cpp
)

add_executable(relative_pose_estimation ${relative_pose_estimation_src})
target_link_libraries(relative_pose_estimation
    ${OpenCV_LIBRARIES}
    )

set(distance_estimation_src
    src/distance_estimation.cpp
)

add_executable(distance_estimation ${distance_estimation_src})
target_link_libraries(distance_estimation
    ${OpenCV_LIBRARIES}
    )

target_compile_options(generate_marker
    PRIVATE -O3 -std=c++11
    )


