# CMake 最低版本要求
cmake_minimum_required(VERSION 3.10)

project(RMCV_TRT)

#if(EXISTS /usr/bin/clang AND EXISTS /usr/bin/clang++)
#   set(CMAKE_C_COMPILER /usr/bin/clang)
#   set(CMAKE_CXX_COMPILER /usr/bin/clang++)
#endif()

add_definitions(-std=c++11)
add_definitions(-DAPI_EXPORTS)
option(CUDA_USE_STATIC_CUDA_RUNTIME off)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR})
# set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set(CMAKE_PREFIX_PATH "/usr/lib/aarch64-linux-gnu")

# 库文件的放置目录
set(MV_SDK /home/lsn/vision_lib/linuxSDK_V2.1.0.28/lib/x64/libMVSDK.so)
set(SDK_DIR /home/lsn/vision_lib/Galaxy_Linux-x86_Gige-U3_32bits-64bits_1.2.2104.9022/Galaxy_camera/lib/x86_64/libgxiapi.so)
# C++11 标准、o3优化、多线程
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3 -Wno-sign-compare  -Wno-deprecated-declarations -pthread -Wall -Ofast -g -Wfatal-errors -D_MWAITXINTRIN_H_INCLUDED")

set(OpenCV_DIR "/home/lsn/opencv-4.5.5/build")
# cuda
find_package(CUDA REQUIRED)
include_directories(/usr/local/cuda/include)
link_directories(/usr/local/cuda/lib64)
# tensorrt
include_directories(/home/lsn/TensorRT-8.5.1.7/include)
link_directories(/home/lsn/TensorRT-8.5.1.7/lib)

find_package(OpenCV  REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(fmt REQUIRED)
# Ceres
find_package(Ceres REQUIRED)
file(GLOB_RECURSE "/usr/lib/python3.8/config-x86_64-linux-gnu/*.so")
# link the head files
include_directories("/home/lsn/vision_lib/eigen3")#new
include_directories(${OpenCV_INCLUDE_DIRS} )
include_directories(${TensorRT_INCLUDE_DIRS})
#include_directories("/usr/src/tensorrt/samples/common")
include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${PROJECT_SOURCE_DIR}/DaHeng)
include_directories(${PROJECT_SOURCE_DIR}/MidVison)
include_directories(${PROJECT_SOURCE_DIR}/common)

# link the source files
aux_source_directory(./src SRC_FILES)
aux_source_directory(./DaHeng SRC_FILES)


cuda_add_executable(${PROJECT_NAME}  ${SRC_FILES} ./src/main.cpp ./include/preprocess.cu)

target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})#new
target_link_libraries(${PROJECT_NAME} ${SDK_DIR})
target_link_libraries(${PROJECT_NAME} ${MV_SDK})
target_link_libraries(${PROJECT_NAME} ${CERES_LIBRARIES})
target_link_libraries(${PROJECT_NAME} nvinfer)
target_link_libraries(${PROJECT_NAME} cudart)
target_link_libraries(${PROJECT_NAME} fmt::fmt nvonnxparser cuda)
