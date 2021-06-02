cmake_minimum_required(VERSION 2.8.3)
project(path_practice)
add_compile_options(-std=c++11)

## Compile as C++11, supported in ROS Kinetic and newer

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  core_msgs
  message_generation
  geometry_msgs
  visualization_msgs
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
)

find_package( OpenCV REQUIRED )
find_package( cv_bridge REQUIRED )

catkin_package(
#  INCLUDE_DIRS include
#  CATKIN_DEPENDS roscpp
#  std_msgs
#  core_msgs
#  visualization_msgs
#  DEPENDS system_lib
)
include_directories(
  ${OpenCV_INCLUDE_DIRS}
  ${cv_bridge_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  #include
)
add_executable(path_practice2 src/path_practice2.cpp)
add_dependencies(path_practice2 core_msgs_generate_messages_cpp)

target_link_libraries(path_practice2
  ${catkin_LIBRARIES} ${OpenCV_LIBS} ${cv_bridge_LIBRARIES}
)
