cmake_minimum_required(VERSION 3.0)
project(video C CXX)
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR}/lib)
find_package(CUDA QUIET REQUIRED)

set(CMAKE_CXX_STANDARD 11)
include_directories(/usr/local/include)
include_directories(/usr/include)
include_directories(NvCodec)
include_directories(include)
include_directories(/usr/local/cuda/include)
set(DEPENCES_LIBS
    gflags
    avutil
    avformat
    avcodec
    swscale
    pthread
    )
  find_package(OpenCV QUIET REQUIRED)
 list(APPEND DEPENCES_LIBS opencv_core opencv_highgui opencv_imgproc opencv_imgcodecs opencv_dnn opencv_video)
#  add_subdirectory(learn_decode)
add_executable(video VideoCapture.cpp)
target_link_libraries(video ${DEPENCES_LIBS})