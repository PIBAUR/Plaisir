execute_process(COMMAND "/home/serveur/catkin_ws/build/neato_robot_v1.1/neato_driver/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/serveur/catkin_ws/build/neato_robot_v1.1/neato_driver/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
