cmake_minimum_required(VERSION 3.10)
project(booster_k1_whitelines_detector_zed2i)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)
find_package(ZED 3.8 REQUIRED)
find_package(CUDA REQUIRED)
find_package(starkit_localization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${ZED_INCLUDE_DIRS}
)

add_executable(white_lines_and_field_plane_detector_node
  WhiteLinesAndFieldPlaneDetectorNode.cpp
  WhiteLinesZedDetector.cpp
  LineSizePixProviderZed.cpp
)
ament_target_dependencies(white_lines_and_field_plane_detector_node
  rclcpp
  sensor_msgs
  nav_msgs
  cv_bridge
  image_transport
  OpenCV
  CUDA
  starkit_localization_msgs
  tf2_ros
  tf2_geometry_msgs
)
target_link_libraries(white_lines_and_field_plane_detector_node
  ${OpenCV_LIBS}
  ${ZED_LIBRARIES}
)

install(TARGETS white_lines_and_field_plane_detector_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package() 