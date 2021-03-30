cmake_minimum_required(VERSION 3.5)
project(calibrate_lidar_camera)

set(CMAKE_CXX_STANDARD 11)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
find_package(gflags REQUIRED)
include_directories(${gflags_INCLUDE_DIR})
find_package(Glog REQUIRED)
include_directories(${GLOG_INCLUDE_DIRS})
find_package(Ceres REQUIRED)
include_directories(${CERES_INCLUDE_DIRS})
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
find_package(jsoncpp REQUIRED)
include_directories(${JSONCPP_INCLUDE_DIR})
find_package(Boost COMPONENTS filesystem)
include_directories(${Boost_INCLUDE_DIRS})

add_executable(calibrate_lidar_camera
        src/main.cc
        src/lidar_camera_calibrator.cc
        src/lidar_camera_calibrator.h
        src/llc_reprojection_factor.h
        src/correspondence_factor_base.h
        src/ppc_reprojection_factor.h
        src/util/lidar_trajectory.cc
        src/util/lidar_trajectory.h
        src/util/lidar_frame.cc
        src/util/lidar_frame.h
        src/util/system_calib.cc
        src/util/system_calib.h
        src/util/line_segment_3d.cc
        src/util/line_segment_3d.h
        src/util/line_segment_2d.cc
        src/util/line_segment_2d.h
        )
target_link_libraries(calibrate_lidar_camera
        glog
        jsoncpp
        opencv_world
        ${gflags_LIBRARIES}
        ${Boost_LIBRARIES}
        ${CERES_LIBRARIES}
        )
