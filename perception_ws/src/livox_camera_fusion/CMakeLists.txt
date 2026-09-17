cmake_minimum_required(VERSION 3.0.2)
project(livox_camera_fusion)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  cv_bridge
  pcl_ros
  pcl_conversions
  message_filters
  detect_msgs
  dynamic_reconfigure
)

find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)

generate_dynamic_reconfigure_options(
  cfg/LidarClustering.cfg
)

catkin_package(
  CATKIN_DEPENDS
    roscpp rospy std_msgs sensor_msgs geometry_msgs
    cv_bridge pcl_ros pcl_conversions message_filters
    detect_msgs dynamic_reconfigure
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node
  src/main.cpp
  src/livox_camera_fusion.cpp
)

add_dependencies(${PROJECT_NAME}_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_gencfg
)

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
)

install(TARGETS ${PROJECT_NAME}_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  OPTIONAL
)
