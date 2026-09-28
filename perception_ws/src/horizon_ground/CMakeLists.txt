cmake_minimum_required(VERSION 3.0.2)
project(horizon_ground)

add_compile_options(-std=c++14)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(catkin REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core)
find_package(Eigen3 REQUIRED)

catkin_python_setup()

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
)

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

# 퓨전 노드(C++ 링크)와 livox_clustering(파이썬 ctypes)이 함께 쓴다
add_library(${PROJECT_NAME} SHARED src/horizon_ground.cpp)
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBRARIES})

install(TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
