cmake_minimum_required(VERSION 3.10)
project(learn_ffmpeg CXX CUDA)

set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR}/lib)
include_directories(/usr/local/include)
include_directories(/usr/include)
include_directories(include)
include_directories(dlpack/include)
include_directories(/usr/local/cuda/include)
include_directories(NvCodec)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC -std=c++11")
set(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS} -std=c++11")


set(DEPENCES_LIBS
    gflags
    avutil
    avformat
    avcodec
    swscale
    pthread
    cuart
    dl 
    cuda
    )
#  list(APPEND DEPENCES_LIBS opencv_core opencv_highgui opencv_imgproc opencv_imgcodecs opencv_dnn opencv_video)
#  add_subdirectory(learn_decode)
set(CMAKE_SHARED_LINKER_FLAGS -fPIC)
add_library(video SHARED VideoCapture.hpp 
NvCodec/NvDecoder/NvDecoder.cpp NvCodec/NvDecoder/NvDecoder.h Utils/ColorSpace.cu Utils/ColorSpace.h 
Utils/FFmpegDemuxer.h NvCodec/NvDecoder/NvDecoder.h Utils/NvCodecUtils.h Utils/Logger.h )
target_link_libraries(video ${DEPENCES_LIBS})