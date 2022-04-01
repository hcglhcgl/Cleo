project(aruco)
cmake_minimum_required(VERSION 3.0.0)


add_executable(create_aruco create_aruco.cpp)

target_link_libraries(create_aruco ${OpenCV_LIBS})