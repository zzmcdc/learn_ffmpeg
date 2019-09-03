cmake_minimum_required(VERSION 3.0)
project(video C CXX CUDA)
# set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR}/bin)
# set(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR}/lib)
# find_package(CUDA QUIET REQUIRED)
# find_package( OpenCV REQUIRED )
# find_package(PythonLibs 3.5 REQUIRED)
include_directories(${PYTHON_INCLUDE_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

message(STATUS ${OpenCV_INCLUDE_DIRS})
include_directories( ${OpenCV_INCLUDE_DIRS} )

set(CMAKE_CXX_STANDARD 11)
include_directories(/usr/local/include)
include_directories(/usr/include)
include_directories(NvCodec)
include_directories(include)
include_directories(/usr/local/cuda/include)
link_directories(/usr/local/cuda/lib64)
include_directories(${PYTHON_INCLUDE_DIRS} ./pybind11/include)
set(DEPENCES_LIBS
    gflags
    avutil
    avformat
    avcodec
    swscale
    pthread
    cudart 
    nvcuvid
    )

 find_package(OpenCV QUIET REQUIRED)
 add_subdirectory(pybind11)
pybind11_add_module(video VideoCapture.cpp)
target_link_libraries(video PRIVATE ${OpenCV_LIBS} ${DEPENCES_LIBS})