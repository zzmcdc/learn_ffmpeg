cmake_minimum_required(VERSION 3.0.0)
project(imdeocde C CXX)
set(CMAKE_CXX_STANDARD 14)
find_package(PkgConfig REQUIRED)
pkg_check_modules(FFMPEG libavutil libavcodec libavformat libavdevice libavfilter libswscale libswresample libpostproc)
pkg_check_modules(OpenCV opencv4)
include_directories(${FFMPEG_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
set(LIBS ${FFMPEG_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(hello_world hello_world.cc)
target_link_libraries(hello_world ${LIBS})

add_executable(hwacc hwacc.cpp)
target_link_libraries(hwacc ${LIBS})