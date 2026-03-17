# Install script for directory: /home/wego/ros1/catkin_ws/src/inha25-winter-ros/limo_mission

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/wego/ros1/catkin_ws/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Release")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/limo_mission" TYPE FILE FILES "/home/wego/ros1/catkin_ws/devel/include/limo_mission/LaneDetectConfig.h")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/limo_mission" TYPE FILE FILES "/home/wego/ros1/catkin_ws/devel/include/limo_mission/TrafficLightConfig.h")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/limo_mission" TYPE FILE FILES "/home/wego/ros1/catkin_ws/devel/include/limo_mission/RoundaboutConfig.h")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/limo_mission" TYPE FILE FILES "/home/wego/ros1/catkin_ws/devel/include/limo_mission/ObstacleAvoidConfig.h")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/limo_mission" TYPE FILE FILES "/home/wego/ros1/catkin_ws/devel/include/limo_mission/PedestrianConfig.h")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3/dist-packages/limo_mission" TYPE FILE FILES "/home/wego/ros1/catkin_ws/devel/lib/python3/dist-packages/limo_mission/__init__.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/usr/bin/python3" -m compileall "/home/wego/ros1/catkin_ws/devel/lib/python3/dist-packages/limo_mission/cfg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3/dist-packages/limo_mission" TYPE DIRECTORY FILES "/home/wego/ros1/catkin_ws/devel/lib/python3/dist-packages/limo_mission/cfg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/wego/ros1/catkin_ws/build/inha25-winter-ros/limo_mission/catkin_generated/installspace/limo_mission.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/limo_mission/cmake" TYPE FILE FILES
    "/home/wego/ros1/catkin_ws/build/inha25-winter-ros/limo_mission/catkin_generated/installspace/limo_missionConfig.cmake"
    "/home/wego/ros1/catkin_ws/build/inha25-winter-ros/limo_mission/catkin_generated/installspace/limo_missionConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/limo_mission" TYPE FILE FILES "/home/wego/ros1/catkin_ws/src/inha25-winter-ros/limo_mission/package.xml")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/limo_mission" TYPE PROGRAM FILES "/home/wego/ros1/catkin_ws/build/inha25-winter-ros/limo_mission/catkin_generated/installspace/lane_detect_node.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/limo_mission" TYPE PROGRAM FILES "/home/wego/ros1/catkin_ws/build/inha25-winter-ros/limo_mission/catkin_generated/installspace/traffic_light_node.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/limo_mission" TYPE PROGRAM FILES "/home/wego/ros1/catkin_ws/build/inha25-winter-ros/limo_mission/catkin_generated/installspace/roundabout_node.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/limo_mission" TYPE PROGRAM FILES "/home/wego/ros1/catkin_ws/build/inha25-winter-ros/limo_mission/catkin_generated/installspace/obstacle_avoid_node.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/limo_mission" TYPE PROGRAM FILES "/home/wego/ros1/catkin_ws/build/inha25-winter-ros/limo_mission/catkin_generated/installspace/pedestrian_node.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/limo_mission" TYPE PROGRAM FILES "/home/wego/ros1/catkin_ws/build/inha25-winter-ros/limo_mission/catkin_generated/installspace/01_color_filter.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/limo_mission" TYPE PROGRAM FILES "/home/wego/ros1/catkin_ws/build/inha25-winter-ros/limo_mission/catkin_generated/installspace/02_roi.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/limo_mission" TYPE PROGRAM FILES "/home/wego/ros1/catkin_ws/build/inha25-winter-ros/limo_mission/catkin_generated/installspace/03_lidar_distance.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/limo_mission" TYPE PROGRAM FILES "/home/wego/ros1/catkin_ws/build/inha25-winter-ros/limo_mission/catkin_generated/installspace/state_manager.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/limo_mission" TYPE PROGRAM FILES "/home/wego/ros1/catkin_ws/build/inha25-winter-ros/limo_mission/catkin_generated/installspace/modified_traffic_light_node.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/limo_mission" TYPE PROGRAM FILES "/home/wego/ros1/catkin_ws/build/inha25-winter-ros/limo_mission/catkin_generated/installspace/modified_lane_detection.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/limo_mission" TYPE PROGRAM FILES "/home/wego/ros1/catkin_ws/build/inha25-winter-ros/limo_mission/catkin_generated/installspace/modified_parking_node.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/limo_mission" TYPE PROGRAM FILES "/home/wego/ros1/catkin_ws/build/inha25-winter-ros/limo_mission/catkin_generated/installspace/mission_controller.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/limo_mission" TYPE PROGRAM FILES "/home/wego/ros1/catkin_ws/build/inha25-winter-ros/limo_mission/catkin_generated/installspace/bev_overlay_node.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/limo_mission" TYPE PROGRAM FILES "/home/wego/ros1/catkin_ws/build/inha25-winter-ros/limo_mission/catkin_generated/installspace/modified_pedestrian.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/limo_mission" TYPE PROGRAM FILES "/home/wego/ros1/catkin_ws/build/inha25-winter-ros/limo_mission/catkin_generated/installspace/modified_rotary.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/limo_mission" TYPE PROGRAM FILES "/home/wego/ros1/catkin_ws/build/inha25-winter-ros/limo_mission/catkin_generated/installspace/modified_obstacle_avoid_node.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/limo_mission" TYPE PROGRAM FILES "/home/wego/ros1/catkin_ws/build/inha25-winter-ros/limo_mission/catkin_generated/installspace/histogram_obstacle_avoid_node.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/limo_mission" TYPE PROGRAM FILES "/home/wego/ros1/catkin_ws/build/inha25-winter-ros/limo_mission/catkin_generated/installspace/lane_following.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/limo_mission" TYPE PROGRAM FILES "/home/wego/ros1/catkin_ws/build/inha25-winter-ros/limo_mission/catkin_generated/installspace/modified_state_manager.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/limo_mission" TYPE PROGRAM FILES "/home/wego/ros1/catkin_ws/build/inha25-winter-ros/limo_mission/catkin_generated/installspace/final_lane_detection.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/limo_mission" TYPE PROGRAM FILES "/home/wego/ros1/catkin_ws/build/inha25-winter-ros/limo_mission/catkin_generated/installspace/final_pedestrian.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/limo_mission/launch" TYPE DIRECTORY FILES "/home/wego/ros1/catkin_ws/src/inha25-winter-ros/limo_mission/launch/")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/limo_mission/config" TYPE DIRECTORY FILES "/home/wego/ros1/catkin_ws/src/inha25-winter-ros/limo_mission/config/")
endif()

