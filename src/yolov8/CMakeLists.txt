cmake_minimum_required(VERSION 3.0.2)
project(yolov8)

## find catkin and all required components
find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  cv_bridge
  geometry_msgs
  std_msgs
  detect_msgs
)

## export catkin package
catkin_package(
  CATKIN_DEPENDS
    rospy
    sensor_msgs
    cv_bridge
    geometry_msgs
    std_msgs
    detect_msgs
)

## include directories
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## install Python node
install(PROGRAMS
  scripts/yolo_detect.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## install launch files
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

