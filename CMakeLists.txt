project(robobot)
cmake_minimum_required(VERSION 3.0.0)

set (CMAKE_CXX_STANDARD 17)

find_package(OpenCV REQUIRED)
find_package(Threads REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBCAMERA REQUIRED libcamera)

include_directories(${LIBCAMERA_INCLUDE_DIRS} ${OPENCV_INCLUDE_DIRS})
add_executable(takephoto takephoto.cpp)
add_executable(takevideo takevideo.cpp AppleDetector.cpp)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -pedantic -std=c++11 ${EXTRA_CC_FLAGS} -Wno-psabi")
set(CMAKE_C_FLAGS ${CMAKE_C_FLAGS} "-pthread")
add_executable(mission main.cpp urun.cpp ucamera.cpp ubridge.cpp umission.cpp utime.cpp tcpCase.cpp uevent.cpp ujoy.cpp uinfo.cpp umotor.cpp uedge.cpp upose.cpp uirdist.cpp uaccgyro.cpp uaruco.cpp ulibpose2pose.cpp ulib2dline.cpp ulibpose.cpp ulibposev.cpp ucamera_v4l2.cpp)

target_link_libraries(takephoto -llccv ${OpenCV_LIBS})
target_link_libraries(takevideo -llccv ${OpenCV_LIBS})
target_link_libraries(mission ${OpenCV_LIBS} ${CMAKE_THREAD_LIBS_INIT})
install(TARGETS mission RUNTIME DESTINATION bin)