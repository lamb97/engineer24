cmake_minimum_required(VERSION 3.16)
project(engineer)
find_package(OpenCV REQUIRED)
INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIRS})
set(CMAKE_CXX_STANDARD 14)


add_executable(engineer src/src/src/recognize.cpp
        src/src/include/recognize.h
        src/src/test/test1.cpp


)
target_link_libraries(engineer ${OpenCV_LIBS})

# 为使用 sophus，需要使用find_package命令找到它
find_package(Sophus REQUIRED)
include_directories(${Sophus_INCLUDE_DIRS})

# Eigen
include_directories("/usr/local/include/eigen3")

target_link_libraries(engineer ${Sophus_LIBRARIES} fmt)
# 初始化头文件包含目录
include_directories(
        ${PROJECT_SOURCE_DIR}/src/uart/include
        ${PROJECT_SOURCE_DIR}/src/src/include
        ${PROJECT_SOURCE_DIR}/src/logger/include
        ${OpenCV_INCLUDE_DIRS})
# 加载系统子模块
add_subdirectory(src/src)
add_subdirectory(src/camera)
add_subdirectory(src/uart)
add_subdirectory(src/logger)