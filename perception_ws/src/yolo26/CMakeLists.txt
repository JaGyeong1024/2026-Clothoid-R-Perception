cmake_minimum_required(VERSION 3.0.2)
project(yolo26)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  cv_bridge
  geometry_msgs
  std_msgs
  detect_msgs
)

catkin_package(
  CATKIN_DEPENDS
    rospy
    sensor_msgs
    cv_bridge
    geometry_msgs
    std_msgs
    detect_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(PROGRAMS
  scripts/yolo_detect.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
