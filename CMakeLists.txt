cmake_minimum_required(VERSION 3.0.2)
project(pose_kidnapper)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  tf
  tf2
  tf2_ros
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES pose_kidnapper
#  CATKIN_DEPENDS geometry_msgs roscpp tf tf2 tf2_ros
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(random_kidnap src/random_kidnap.cpp)

target_link_libraries(random_kidnap
  ${catkin_LIBRARIES}
)