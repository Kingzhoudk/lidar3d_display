cmake_minimum_required(VERSION 3.5.1)
project(lidar3d)
#set(BUILD_TYPE Release)
set(BUILD_TYPE RelWithDebInfo)

SET(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} "-std=c++14 -pthread")
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Sophus REQUIRED)
find_package(Pangolin 0.4 REQUIRED)
find_package(Glog REQUIRED)
find_package(GFlags REQUIRED)
find_package(Protobuf REQUIRED)


set(PROTO_FILES
        src/config/config.proto)

PROTOBUF_GENERATE_CPP(PROTO_SRC PROTO_HDR ${PROTO_FILES})

add_executable(lidar3d
        src/lidar3d_data/api.h
        src/lidar3d_data/lidar3d.h src/lidar3d_data/lidar3d.cpp
        main.cpp
        src/display/display.cpp src/display/display.h)

target_link_libraries(lidar3d
        glog
        gflags
        ${OpenCV_LIBS}
        ${PROTOBUF_LIBRARIES}
        ${Sophus_LIBRARIES}
        ${Pangolin_LIBRARIES}
        )

target_link_libraries(lidar3d hps3d64.so) # 显示指定链接动态库
