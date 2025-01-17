cmake_minimum_required(VERSION 3.0.2)
project(lidar_filtering_assignment)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  pcl_ros
  pcl_conversions
)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(lidar_filtering_node src/lidar_filtering_node.cpp)
target_link_libraries(lidar_filtering_node ${catkin_LIBRARIES})