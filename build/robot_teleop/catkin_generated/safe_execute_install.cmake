execute_process(COMMAND "/home/jhkim/catkin_ws/Capstone1_2021Spring/build/robot_teleop/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/jhkim/catkin_ws/Capstone1_2021Spring/build/robot_teleop/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
