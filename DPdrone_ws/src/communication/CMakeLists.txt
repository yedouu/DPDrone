cmake_minimum_required(VERSION 2.8.3)
project(communication)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  message_generation
  cv_bridge
  tf
  tf2
  tf2_eigen
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs message_generation cv_bridge tf tf2 tf2_eigen
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/include
)

add_executable(drone_communication src/drone_communication.cpp)
target_link_libraries(drone_communication ${catkin_LIBRARIES})
