cmake_minimum_required(VERSION 2.8.3)
project(localization)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  pcl_conversions
  pcl_ros
  sensor_msgs
  core_msgs
  message_generation
  geometry_msgs
)

add_message_files(
  FILES
  multi_position.msg
  robot_position.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
)

find_package( OpenCV REQUIRED )

catkin_package(
#  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs message_runtime
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  #include
)

add_executable(localization src/localization.cpp)
add_dependencies(localization core_msgs_generate_messages_cpp)
target_link_libraries(localization
  ${catkin_LIBRARIES}
<<<<<<< HEAD
)
=======
)
>>>>>>> 0c7167fe59a1f365e97b0eb787cb8ef87fee1fe9
