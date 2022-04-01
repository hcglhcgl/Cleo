project(aruco)
cmake_minimum_required(VERSION 3.0.0)


add_executable(create_aruco create_aruco.cpp)

target_link_libraries(create_aruco ${OpenCV_LIBS})

add_executable(single_picture single_picture.cpp aruco.cpp)

target_link_libraries(single_picture ${OpenCV_LIBS})

add_executable(aruco_live aruco_live.cpp aruco.cpp)

target_link_libraries(aruco_live -llccv ${OpenCV_LIBS})