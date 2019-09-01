cmake_minimum_required(VERSION 3.10)
project(learn_ffmpeg C CXX)

set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR}/lib)
include_directories(/usr/local/include)
include_directories(/usr/include)

set(DEPENCES_LIBS
    gflags
    avutil
    avformat
    avcodec
    swscale
    pthread
    )
#  list(APPEND DEPENCES_LIBS opencv_core opencv_highgui opencv_imgproc opencv_imgcodecs opencv_dnn opencv_video)
#  add_subdirectory(learn_decode)
message(STATUS $DEPENCES_LIBS)
add_executable(decode_video examples/decode_video.c)
target_link_libraries(decode_video ${DEPENCES_LIBS})