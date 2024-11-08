cmake_minimum_required(VERSION 2.8.3)
project(control)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  message_generation
  cv_bridge
  tf
  tf2
  tf2_eigen
  quadrotor_msgs
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

add_executable(ego_ctrl_v1 src/ego_ctrl_v1.cpp)
target_link_libraries(ego_ctrl_v1 ${catkin_LIBRARIES})

add_executable(fuel_ctrl_v1 src/fuel_ctrl_v1.cpp)
target_link_libraries(fuel_ctrl_v1 ${catkin_LIBRARIES})

add_executable(vins_test src/vins_test.cpp)
target_link_libraries(vins_test ${catkin_LIBRARIES})
