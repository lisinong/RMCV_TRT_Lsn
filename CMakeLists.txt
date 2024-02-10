# CMake 最低版本要求
cmake_minimum_required(VERSION 3.10)

project(RMCV_TRT)

add_definitions(-std=c++11)
add_definitions(-DAPI_EXPORTS)
option(CUDA_USE_STATIC_CUDA_RUNTIME off)
set(CMAKE_CXX_STANDARD 11)
#set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_BUILD_TYPE "debug")
set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR})
FIND_PROGRAM(CCACHE_FOUND ccache) # ccache编译器缓存
IF(CCACHE_FOUND)
    SET_PROPERTY(GLOBAL PROPERTY RULE_LAUNCH_COMPILE ccache)
    SET_PROPERTY(GLOBAL PROPERTY RULE_LAUNCH_LINK ccache)
    MESSAGE("< Use ccache for compiler >")
ENDIF()

# 相机库文件的放置目录
set(MV_SDK /home/lsn/vision_lib/linuxSDK_V2.1.0.28/lib/x64/libMVSDK.so)
set(SDK_DIR /home/lsn/vision_lib/Galaxy_Linux-x86_Gige-U3_32bits-64bits_1.2.2104.9022/Galaxy_camera/lib/x86_64/libgxiapi.so)
set(HIKVISION_CAMERA_DIR "${PROJECT_SOURCE_DIR}/HikVision") # 设置相机SDK工具包目录
SET(YAML_CPP_LIBRARIES /usr/local/lib/libyaml-cpp.so) # 设置yaml-cpp库
# 指定相机动态链接库所在路径
LINK_DIRECTORIES(${HIKVISION_CAMERA_DIR}/64)
# 指定相机库目录
INCLUDE_DIRECTORIES(${HIKVISION_CAMERA_DIR}/include)
INCLUDE_DIRECTORIES(${HIKVISION_CAMERA_DIR}/tool/include)
# 相机拓展工具文件
FILE(GLOB_RECURSE sourcefiles "${HIKVISION_CAMERA_DIR}/src/*.cpp" "${HIKVISION_CAMERA_DIR}/tool/src/*.cpp")
# C++11 标准、o3优化、多线程
set(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -std=c++11 -O3 -Wno-sign-compare  -Wno-deprecated-declarations -pthread -Wall -Ofast -fopenmp -g -Wfatal-errors -D_MWAITXINTRIN_H_INCLUDED")# 定义Release编译参数
SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -std=c++11 -Wall -g2 -ggdb -pthread -fopenmp -g -Wfatal-errors -D_MWAITXINTRIN_H_INCLUDED") # 定义Debug编译参数


#opencv
#set(OpenCV_DIR "/home/lsn/opencv-4.5.5/build")

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

# link the head files
include_directories("/home/lsn/vision_lib/eigen3")
include_directories(${OpenCV_INCLUDE_DIRS} )
include_directories(${TensorRT_INCLUDE_DIRS})
include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${PROJECT_SOURCE_DIR}/DaHeng)
include_directories(${PROJECT_SOURCE_DIR}/MidVison)
include_directories(${PROJECT_SOURCE_DIR}/common)

# link the source files
aux_source_directory(./src SRC_FILES)
aux_source_directory(./DaHeng SRC_FILES)


cuda_add_executable(${PROJECT_NAME}  ${SRC_FILES} ${sourcefiles} ./src/main.cpp ./include/preprocess.cu )


target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})#new
target_link_libraries(${PROJECT_NAME} ${SDK_DIR})
target_link_libraries(${PROJECT_NAME} ${MV_SDK})
target_link_libraries(${PROJECT_NAME} ${CERES_LIBRARIES})
target_link_libraries(${PROJECT_NAME} nvinfer)
target_link_libraries(${PROJECT_NAME} cudart)
target_link_libraries(${PROJECT_NAME} fmt::fmt  nvonnxparser cuda)
# 将spdlog的include目录添加到你的项目中
target_include_directories(${PROJECT_NAME} PRIVATE spdlog/include)  
target_link_libraries(${PROJECT_NAME} ${YAML_CPP_LIBRARIES})
target_link_libraries(${PROJECT_NAME} MvCameraControl)