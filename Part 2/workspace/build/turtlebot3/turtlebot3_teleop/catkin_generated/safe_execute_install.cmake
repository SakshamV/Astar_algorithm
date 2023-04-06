execute_process(COMMAND "/home/saksham/Downloads/planning_p3v2/workspace/build/turtlebot3/turtlebot3_teleop/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/saksham/Downloads/planning_p3v2/workspace/build/turtlebot3/turtlebot3_teleop/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
