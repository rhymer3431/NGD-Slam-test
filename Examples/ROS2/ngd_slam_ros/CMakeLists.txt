cmake_minimum_required(VERSION 3.8)
project(ngd_slam_ros)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(CURL REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(OpenCV 4.4 REQUIRED)
find_package(Pangolin REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Boost REQUIRED COMPONENTS serialization system)
find_package(OpenSSL REQUIRED)

get_filename_component(NGD_SLAM_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../.." ABSOLUTE)

find_path(REALSENSE_INCLUDE_DIRS
  NAMES librealsense2/rs.hpp
  HINTS
    "${NGD_SLAM_ROOT}/.deps/realsense2/include"
    "${NGD_SLAM_ROOT}/.deps/librealsense-src/include"
)
find_library(REALSENSE_LIBRARIES
  NAMES realsense2 librealsense2
  HINTS
    "${NGD_SLAM_ROOT}/.deps/realsense2/lib"
    "${NGD_SLAM_ROOT}/.deps/librealsense-build"
    "${NGD_SLAM_ROOT}/.deps/librealsense-build/Release"
)

if(NOT REALSENSE_INCLUDE_DIRS OR NOT REALSENSE_LIBRARIES)
  message(FATAL_ERROR
    "RealSense SDK not found. Build librealsense in .deps or install realsense2 development files.")
endif()

set(NGD_SLAM_LIBRARY "${NGD_SLAM_ROOT}/lib/libORB_SLAM3.so")
set(DBOW2_LIBRARY "${NGD_SLAM_ROOT}/Thirdparty/DBoW2/lib/libDBoW2.so")
set(G2O_LIBRARY "${NGD_SLAM_ROOT}/Thirdparty/g2o/lib/libg2o.so")

if(NOT EXISTS "${NGD_SLAM_LIBRARY}")
  message(FATAL_ERROR
    "Missing ${NGD_SLAM_LIBRARY}. Build NGD-SLAM first from the repository root with ./build.sh")
endif()

add_executable(ngd_slam_rgbd_node
  src/ngd_slam_rgbd_node.cpp
)

add_executable(realsense_d455_rgbd_node
  src/realsense_d455_rgbd_node.cpp
)

target_include_directories(ngd_slam_rgbd_node PRIVATE
  ${NGD_SLAM_ROOT}
  ${NGD_SLAM_ROOT}/include
  ${NGD_SLAM_ROOT}/include/CameraModels
  ${NGD_SLAM_ROOT}/Thirdparty/Sophus
  ${Pangolin_INCLUDE_DIRS}
)

target_include_directories(realsense_d455_rgbd_node PRIVATE
  ${REALSENSE_INCLUDE_DIRS}
)

ament_target_dependencies(ngd_slam_rgbd_node
  cv_bridge
  geometry_msgs
  message_filters
  rclcpp
  sensor_msgs
  std_msgs
)

ament_target_dependencies(realsense_d455_rgbd_node
  rclcpp
  sensor_msgs
)

target_link_libraries(ngd_slam_rgbd_node
  ${NGD_SLAM_LIBRARY}
  ${DBOW2_LIBRARY}
  ${G2O_LIBRARY}
  ${OpenCV_LIBS}
  ${Pangolin_LIBRARIES}
  Boost::serialization
  Boost::system
  CURL::libcurl
  OpenSSL::Crypto
)

target_link_libraries(realsense_d455_rgbd_node
  ${REALSENSE_LIBRARIES}
)

install(TARGETS
  ngd_slam_rgbd_node
  realsense_d455_rgbd_node
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/publish_tum_rgbd.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

install(FILES
  package.xml
  README.md
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
