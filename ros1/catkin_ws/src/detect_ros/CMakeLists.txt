cmake_minimum_required(VERSION 3.0.2)
project(detect_ros)

find_package(PCL REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf
  nav_msgs
  geometry_msgs
  sensor_msgs
  pcl_ros
  laser_geometry 
  ar_track_alvar_msgs
)

catkin_package()

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)


add_executable(lifter src/lifter.cpp)
target_link_libraries(lifter  ${catkin_LIBRARIES})

