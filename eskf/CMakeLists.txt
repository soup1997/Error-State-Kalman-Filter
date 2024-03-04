cmake_minimum_required(VERSION 3.0.2)
project(eskf)

find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  sensor_msgs
  std_msgs
  tf2
  roscpp
)

find_package(Eigen3 REQUIRED)
find_package(OpenMP)

if(OPENMP_FOUND)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  set(OpenMP_LIBRARIES gomp)
endif()


generate_messages(
  DEPENDENCIES
  sensor_msgs
  std_msgs
  geometry_msgs
  nav_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES eskf
  CATKIN_DEPENDS nav_msgs roscpp sensor_msgs std_msgs tf2
  DEPENDS Eigen3
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/eskf_node.cpp src/eskf.cpp src/util.cpp)

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
  ${OpenMP_LIBRARIES}
)