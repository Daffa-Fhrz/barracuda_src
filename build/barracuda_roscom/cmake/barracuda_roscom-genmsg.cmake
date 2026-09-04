# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "barracuda_roscom: 8 messages, 0 services")

set(MSG_I_FLAGS "-Ibarracuda_roscom:/home/sogo/sogo_ws/src/barracuda_roscom/msg;-Igeometry_msgs:/opt/ros/noetic/share/geometry_msgs/cmake/../msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(barracuda_roscom_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/sogo/sogo_ws/src/barracuda_roscom/msg/speedRobot.msg" NAME_WE)
add_custom_target(_barracuda_roscom_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "barracuda_roscom" "/home/sogo/sogo_ws/src/barracuda_roscom/msg/speedRobot.msg" ""
)

get_filename_component(_filename "/home/sogo/sogo_ws/src/barracuda_roscom/msg/frontInfo.msg" NAME_WE)
add_custom_target(_barracuda_roscom_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "barracuda_roscom" "/home/sogo/sogo_ws/src/barracuda_roscom/msg/frontInfo.msg" ""
)

get_filename_component(_filename "/home/sogo/sogo_ws/src/barracuda_roscom/msg/ballInfo.msg" NAME_WE)
add_custom_target(_barracuda_roscom_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "barracuda_roscom" "/home/sogo/sogo_ws/src/barracuda_roscom/msg/ballInfo.msg" ""
)

get_filename_component(_filename "/home/sogo/sogo_ws/src/barracuda_roscom/msg/ballTravel.msg" NAME_WE)
add_custom_target(_barracuda_roscom_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "barracuda_roscom" "/home/sogo/sogo_ws/src/barracuda_roscom/msg/ballTravel.msg" ""
)

get_filename_component(_filename "/home/sogo/sogo_ws/src/barracuda_roscom/msg/currentPose.msg" NAME_WE)
add_custom_target(_barracuda_roscom_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "barracuda_roscom" "/home/sogo/sogo_ws/src/barracuda_roscom/msg/currentPose.msg" ""
)

get_filename_component(_filename "/home/sogo/sogo_ws/src/barracuda_roscom/msg/penggiring.msg" NAME_WE)
add_custom_target(_barracuda_roscom_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "barracuda_roscom" "/home/sogo/sogo_ws/src/barracuda_roscom/msg/penggiring.msg" ""
)

get_filename_component(_filename "/home/sogo/sogo_ws/src/barracuda_roscom/msg/ballCatch.msg" NAME_WE)
add_custom_target(_barracuda_roscom_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "barracuda_roscom" "/home/sogo/sogo_ws/src/barracuda_roscom/msg/ballCatch.msg" ""
)

get_filename_component(_filename "/home/sogo/sogo_ws/src/barracuda_roscom/msg/omniInfo.msg" NAME_WE)
add_custom_target(_barracuda_roscom_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "barracuda_roscom" "/home/sogo/sogo_ws/src/barracuda_roscom/msg/omniInfo.msg" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(barracuda_roscom
  "/home/sogo/sogo_ws/src/barracuda_roscom/msg/speedRobot.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/barracuda_roscom
)
_generate_msg_cpp(barracuda_roscom
  "/home/sogo/sogo_ws/src/barracuda_roscom/msg/frontInfo.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/barracuda_roscom
)
_generate_msg_cpp(barracuda_roscom
  "/home/sogo/sogo_ws/src/barracuda_roscom/msg/ballInfo.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/barracuda_roscom
)
_generate_msg_cpp(barracuda_roscom
  "/home/sogo/sogo_ws/src/barracuda_roscom/msg/ballTravel.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/barracuda_roscom
)
_generate_msg_cpp(barracuda_roscom
  "/home/sogo/sogo_ws/src/barracuda_roscom/msg/currentPose.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/barracuda_roscom
)
_generate_msg_cpp(barracuda_roscom
  "/home/sogo/sogo_ws/src/barracuda_roscom/msg/penggiring.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/barracuda_roscom
)
_generate_msg_cpp(barracuda_roscom
  "/home/sogo/sogo_ws/src/barracuda_roscom/msg/ballCatch.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/barracuda_roscom
)
_generate_msg_cpp(barracuda_roscom
  "/home/sogo/sogo_ws/src/barracuda_roscom/msg/omniInfo.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/barracuda_roscom
)

### Generating Services

### Generating Module File
_generate_module_cpp(barracuda_roscom
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/barracuda_roscom
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(barracuda_roscom_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(barracuda_roscom_generate_messages barracuda_roscom_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/sogo/sogo_ws/src/barracuda_roscom/msg/speedRobot.msg" NAME_WE)
add_dependencies(barracuda_roscom_generate_messages_cpp _barracuda_roscom_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sogo/sogo_ws/src/barracuda_roscom/msg/frontInfo.msg" NAME_WE)
add_dependencies(barracuda_roscom_generate_messages_cpp _barracuda_roscom_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sogo/sogo_ws/src/barracuda_roscom/msg/ballInfo.msg" NAME_WE)
add_dependencies(barracuda_roscom_generate_messages_cpp _barracuda_roscom_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sogo/sogo_ws/src/barracuda_roscom/msg/ballTravel.msg" NAME_WE)
add_dependencies(barracuda_roscom_generate_messages_cpp _barracuda_roscom_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sogo/sogo_ws/src/barracuda_roscom/msg/currentPose.msg" NAME_WE)
add_dependencies(barracuda_roscom_generate_messages_cpp _barracuda_roscom_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sogo/sogo_ws/src/barracuda_roscom/msg/penggiring.msg" NAME_WE)
add_dependencies(barracuda_roscom_generate_messages_cpp _barracuda_roscom_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sogo/sogo_ws/src/barracuda_roscom/msg/ballCatch.msg" NAME_WE)
add_dependencies(barracuda_roscom_generate_messages_cpp _barracuda_roscom_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sogo/sogo_ws/src/barracuda_roscom/msg/omniInfo.msg" NAME_WE)
add_dependencies(barracuda_roscom_generate_messages_cpp _barracuda_roscom_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(barracuda_roscom_gencpp)
add_dependencies(barracuda_roscom_gencpp barracuda_roscom_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS barracuda_roscom_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(barracuda_roscom
  "/home/sogo/sogo_ws/src/barracuda_roscom/msg/speedRobot.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/barracuda_roscom
)
_generate_msg_eus(barracuda_roscom
  "/home/sogo/sogo_ws/src/barracuda_roscom/msg/frontInfo.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/barracuda_roscom
)
_generate_msg_eus(barracuda_roscom
  "/home/sogo/sogo_ws/src/barracuda_roscom/msg/ballInfo.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/barracuda_roscom
)
_generate_msg_eus(barracuda_roscom
  "/home/sogo/sogo_ws/src/barracuda_roscom/msg/ballTravel.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/barracuda_roscom
)
_generate_msg_eus(barracuda_roscom
  "/home/sogo/sogo_ws/src/barracuda_roscom/msg/currentPose.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/barracuda_roscom
)
_generate_msg_eus(barracuda_roscom
  "/home/sogo/sogo_ws/src/barracuda_roscom/msg/penggiring.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/barracuda_roscom
)
_generate_msg_eus(barracuda_roscom
  "/home/sogo/sogo_ws/src/barracuda_roscom/msg/ballCatch.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/barracuda_roscom
)
_generate_msg_eus(barracuda_roscom
  "/home/sogo/sogo_ws/src/barracuda_roscom/msg/omniInfo.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/barracuda_roscom
)

### Generating Services

### Generating Module File
_generate_module_eus(barracuda_roscom
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/barracuda_roscom
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(barracuda_roscom_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(barracuda_roscom_generate_messages barracuda_roscom_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/sogo/sogo_ws/src/barracuda_roscom/msg/speedRobot.msg" NAME_WE)
add_dependencies(barracuda_roscom_generate_messages_eus _barracuda_roscom_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sogo/sogo_ws/src/barracuda_roscom/msg/frontInfo.msg" NAME_WE)
add_dependencies(barracuda_roscom_generate_messages_eus _barracuda_roscom_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sogo/sogo_ws/src/barracuda_roscom/msg/ballInfo.msg" NAME_WE)
add_dependencies(barracuda_roscom_generate_messages_eus _barracuda_roscom_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sogo/sogo_ws/src/barracuda_roscom/msg/ballTravel.msg" NAME_WE)
add_dependencies(barracuda_roscom_generate_messages_eus _barracuda_roscom_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sogo/sogo_ws/src/barracuda_roscom/msg/currentPose.msg" NAME_WE)
add_dependencies(barracuda_roscom_generate_messages_eus _barracuda_roscom_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sogo/sogo_ws/src/barracuda_roscom/msg/penggiring.msg" NAME_WE)
add_dependencies(barracuda_roscom_generate_messages_eus _barracuda_roscom_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sogo/sogo_ws/src/barracuda_roscom/msg/ballCatch.msg" NAME_WE)
add_dependencies(barracuda_roscom_generate_messages_eus _barracuda_roscom_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sogo/sogo_ws/src/barracuda_roscom/msg/omniInfo.msg" NAME_WE)
add_dependencies(barracuda_roscom_generate_messages_eus _barracuda_roscom_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(barracuda_roscom_geneus)
add_dependencies(barracuda_roscom_geneus barracuda_roscom_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS barracuda_roscom_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(barracuda_roscom
  "/home/sogo/sogo_ws/src/barracuda_roscom/msg/speedRobot.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/barracuda_roscom
)
_generate_msg_lisp(barracuda_roscom
  "/home/sogo/sogo_ws/src/barracuda_roscom/msg/frontInfo.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/barracuda_roscom
)
_generate_msg_lisp(barracuda_roscom
  "/home/sogo/sogo_ws/src/barracuda_roscom/msg/ballInfo.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/barracuda_roscom
)
_generate_msg_lisp(barracuda_roscom
  "/home/sogo/sogo_ws/src/barracuda_roscom/msg/ballTravel.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/barracuda_roscom
)
_generate_msg_lisp(barracuda_roscom
  "/home/sogo/sogo_ws/src/barracuda_roscom/msg/currentPose.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/barracuda_roscom
)
_generate_msg_lisp(barracuda_roscom
  "/home/sogo/sogo_ws/src/barracuda_roscom/msg/penggiring.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/barracuda_roscom
)
_generate_msg_lisp(barracuda_roscom
  "/home/sogo/sogo_ws/src/barracuda_roscom/msg/ballCatch.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/barracuda_roscom
)
_generate_msg_lisp(barracuda_roscom
  "/home/sogo/sogo_ws/src/barracuda_roscom/msg/omniInfo.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/barracuda_roscom
)

### Generating Services

### Generating Module File
_generate_module_lisp(barracuda_roscom
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/barracuda_roscom
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(barracuda_roscom_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(barracuda_roscom_generate_messages barracuda_roscom_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/sogo/sogo_ws/src/barracuda_roscom/msg/speedRobot.msg" NAME_WE)
add_dependencies(barracuda_roscom_generate_messages_lisp _barracuda_roscom_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sogo/sogo_ws/src/barracuda_roscom/msg/frontInfo.msg" NAME_WE)
add_dependencies(barracuda_roscom_generate_messages_lisp _barracuda_roscom_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sogo/sogo_ws/src/barracuda_roscom/msg/ballInfo.msg" NAME_WE)
add_dependencies(barracuda_roscom_generate_messages_lisp _barracuda_roscom_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sogo/sogo_ws/src/barracuda_roscom/msg/ballTravel.msg" NAME_WE)
add_dependencies(barracuda_roscom_generate_messages_lisp _barracuda_roscom_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sogo/sogo_ws/src/barracuda_roscom/msg/currentPose.msg" NAME_WE)
add_dependencies(barracuda_roscom_generate_messages_lisp _barracuda_roscom_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sogo/sogo_ws/src/barracuda_roscom/msg/penggiring.msg" NAME_WE)
add_dependencies(barracuda_roscom_generate_messages_lisp _barracuda_roscom_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sogo/sogo_ws/src/barracuda_roscom/msg/ballCatch.msg" NAME_WE)
add_dependencies(barracuda_roscom_generate_messages_lisp _barracuda_roscom_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sogo/sogo_ws/src/barracuda_roscom/msg/omniInfo.msg" NAME_WE)
add_dependencies(barracuda_roscom_generate_messages_lisp _barracuda_roscom_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(barracuda_roscom_genlisp)
add_dependencies(barracuda_roscom_genlisp barracuda_roscom_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS barracuda_roscom_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(barracuda_roscom
  "/home/sogo/sogo_ws/src/barracuda_roscom/msg/speedRobot.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/barracuda_roscom
)
_generate_msg_nodejs(barracuda_roscom
  "/home/sogo/sogo_ws/src/barracuda_roscom/msg/frontInfo.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/barracuda_roscom
)
_generate_msg_nodejs(barracuda_roscom
  "/home/sogo/sogo_ws/src/barracuda_roscom/msg/ballInfo.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/barracuda_roscom
)
_generate_msg_nodejs(barracuda_roscom
  "/home/sogo/sogo_ws/src/barracuda_roscom/msg/ballTravel.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/barracuda_roscom
)
_generate_msg_nodejs(barracuda_roscom
  "/home/sogo/sogo_ws/src/barracuda_roscom/msg/currentPose.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/barracuda_roscom
)
_generate_msg_nodejs(barracuda_roscom
  "/home/sogo/sogo_ws/src/barracuda_roscom/msg/penggiring.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/barracuda_roscom
)
_generate_msg_nodejs(barracuda_roscom
  "/home/sogo/sogo_ws/src/barracuda_roscom/msg/ballCatch.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/barracuda_roscom
)
_generate_msg_nodejs(barracuda_roscom
  "/home/sogo/sogo_ws/src/barracuda_roscom/msg/omniInfo.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/barracuda_roscom
)

### Generating Services

### Generating Module File
_generate_module_nodejs(barracuda_roscom
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/barracuda_roscom
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(barracuda_roscom_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(barracuda_roscom_generate_messages barracuda_roscom_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/sogo/sogo_ws/src/barracuda_roscom/msg/speedRobot.msg" NAME_WE)
add_dependencies(barracuda_roscom_generate_messages_nodejs _barracuda_roscom_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sogo/sogo_ws/src/barracuda_roscom/msg/frontInfo.msg" NAME_WE)
add_dependencies(barracuda_roscom_generate_messages_nodejs _barracuda_roscom_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sogo/sogo_ws/src/barracuda_roscom/msg/ballInfo.msg" NAME_WE)
add_dependencies(barracuda_roscom_generate_messages_nodejs _barracuda_roscom_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sogo/sogo_ws/src/barracuda_roscom/msg/ballTravel.msg" NAME_WE)
add_dependencies(barracuda_roscom_generate_messages_nodejs _barracuda_roscom_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sogo/sogo_ws/src/barracuda_roscom/msg/currentPose.msg" NAME_WE)
add_dependencies(barracuda_roscom_generate_messages_nodejs _barracuda_roscom_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sogo/sogo_ws/src/barracuda_roscom/msg/penggiring.msg" NAME_WE)
add_dependencies(barracuda_roscom_generate_messages_nodejs _barracuda_roscom_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sogo/sogo_ws/src/barracuda_roscom/msg/ballCatch.msg" NAME_WE)
add_dependencies(barracuda_roscom_generate_messages_nodejs _barracuda_roscom_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sogo/sogo_ws/src/barracuda_roscom/msg/omniInfo.msg" NAME_WE)
add_dependencies(barracuda_roscom_generate_messages_nodejs _barracuda_roscom_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(barracuda_roscom_gennodejs)
add_dependencies(barracuda_roscom_gennodejs barracuda_roscom_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS barracuda_roscom_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(barracuda_roscom
  "/home/sogo/sogo_ws/src/barracuda_roscom/msg/speedRobot.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/barracuda_roscom
)
_generate_msg_py(barracuda_roscom
  "/home/sogo/sogo_ws/src/barracuda_roscom/msg/frontInfo.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/barracuda_roscom
)
_generate_msg_py(barracuda_roscom
  "/home/sogo/sogo_ws/src/barracuda_roscom/msg/ballInfo.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/barracuda_roscom
)
_generate_msg_py(barracuda_roscom
  "/home/sogo/sogo_ws/src/barracuda_roscom/msg/ballTravel.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/barracuda_roscom
)
_generate_msg_py(barracuda_roscom
  "/home/sogo/sogo_ws/src/barracuda_roscom/msg/currentPose.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/barracuda_roscom
)
_generate_msg_py(barracuda_roscom
  "/home/sogo/sogo_ws/src/barracuda_roscom/msg/penggiring.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/barracuda_roscom
)
_generate_msg_py(barracuda_roscom
  "/home/sogo/sogo_ws/src/barracuda_roscom/msg/ballCatch.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/barracuda_roscom
)
_generate_msg_py(barracuda_roscom
  "/home/sogo/sogo_ws/src/barracuda_roscom/msg/omniInfo.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/barracuda_roscom
)

### Generating Services

### Generating Module File
_generate_module_py(barracuda_roscom
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/barracuda_roscom
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(barracuda_roscom_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(barracuda_roscom_generate_messages barracuda_roscom_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/sogo/sogo_ws/src/barracuda_roscom/msg/speedRobot.msg" NAME_WE)
add_dependencies(barracuda_roscom_generate_messages_py _barracuda_roscom_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sogo/sogo_ws/src/barracuda_roscom/msg/frontInfo.msg" NAME_WE)
add_dependencies(barracuda_roscom_generate_messages_py _barracuda_roscom_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sogo/sogo_ws/src/barracuda_roscom/msg/ballInfo.msg" NAME_WE)
add_dependencies(barracuda_roscom_generate_messages_py _barracuda_roscom_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sogo/sogo_ws/src/barracuda_roscom/msg/ballTravel.msg" NAME_WE)
add_dependencies(barracuda_roscom_generate_messages_py _barracuda_roscom_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sogo/sogo_ws/src/barracuda_roscom/msg/currentPose.msg" NAME_WE)
add_dependencies(barracuda_roscom_generate_messages_py _barracuda_roscom_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sogo/sogo_ws/src/barracuda_roscom/msg/penggiring.msg" NAME_WE)
add_dependencies(barracuda_roscom_generate_messages_py _barracuda_roscom_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sogo/sogo_ws/src/barracuda_roscom/msg/ballCatch.msg" NAME_WE)
add_dependencies(barracuda_roscom_generate_messages_py _barracuda_roscom_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sogo/sogo_ws/src/barracuda_roscom/msg/omniInfo.msg" NAME_WE)
add_dependencies(barracuda_roscom_generate_messages_py _barracuda_roscom_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(barracuda_roscom_genpy)
add_dependencies(barracuda_roscom_genpy barracuda_roscom_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS barracuda_roscom_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/barracuda_roscom)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/barracuda_roscom
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(barracuda_roscom_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(barracuda_roscom_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/barracuda_roscom)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/barracuda_roscom
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(barracuda_roscom_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(barracuda_roscom_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/barracuda_roscom)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/barracuda_roscom
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(barracuda_roscom_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(barracuda_roscom_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/barracuda_roscom)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/barracuda_roscom
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(barracuda_roscom_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(barracuda_roscom_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/barracuda_roscom)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/barracuda_roscom\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/barracuda_roscom
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(barracuda_roscom_generate_messages_py geometry_msgs_generate_messages_py)
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(barracuda_roscom_generate_messages_py std_msgs_generate_messages_py)
endif()
