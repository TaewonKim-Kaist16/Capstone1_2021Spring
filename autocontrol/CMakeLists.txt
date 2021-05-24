cmake_minimum_required(VERSION 3.0.2)
project(autocontrol)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  core_msgs
  geometry_msgs
  message_generation
  roscpp
  std_msgs
  localization
)

catkin_package(
#  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs message_runtime
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  #include
)

add_executable(autodriving src/autodriving.cpp)
add_dependencies(autodriving core_msgs_generate_messages_cpp)
target_link_libraries(autodriving
  ${catkin_LIBRARIES}
)