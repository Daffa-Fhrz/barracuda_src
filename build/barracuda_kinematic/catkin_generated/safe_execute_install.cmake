execute_process(COMMAND "/home/sogo/sogo_ws/src/build/barracuda_kinematic/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/sogo/sogo_ws/src/build/barracuda_kinematic/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
