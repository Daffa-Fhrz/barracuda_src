# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "barracuda_vision: 2 messages, 0 services")

set(MSG_I_FLAGS "-Ibarracuda_vision:/home/sogo/sogo_ws/src/barracuda_vision/msg;-Isensor_msgs:/opt/ros/noetic/share/sensor_msgs/cmake/../msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/noetic/share/geometry_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(barracuda_vision_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/sogo/sogo_ws/src/barracuda_vision/msg/ballInfo.msg" NAME_WE)
add_custom_target(_barracuda_vision_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "barracuda_vision" "/home/sogo/sogo_ws/src/barracuda_vision/msg/ballInfo.msg" ""
)

get_filename_component(_filename "/home/sogo/sogo_ws/src/barracuda_vision/msg/ballTravel.msg" NAME_WE)
add_custom_target(_barracuda_vision_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "barracuda_vision" "/home/sogo/sogo_ws/src/barracuda_vision/msg/ballTravel.msg" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(barracuda_vision
  "/home/sogo/sogo_ws/src/barracuda_vision/msg/ballInfo.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/barracuda_vision
)
_generate_msg_cpp(barracuda_vision
  "/home/sogo/sogo_ws/src/barracuda_vision/msg/ballTravel.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/barracuda_vision
)

### Generating Services

### Generating Module File
_generate_module_cpp(barracuda_vision
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/barracuda_vision
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(barracuda_vision_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(barracuda_vision_generate_messages barracuda_vision_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/sogo/sogo_ws/src/barracuda_vision/msg/ballInfo.msg" NAME_WE)
add_dependencies(barracuda_vision_generate_messages_cpp _barracuda_vision_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sogo/sogo_ws/src/barracuda_vision/msg/ballTravel.msg" NAME_WE)
add_dependencies(barracuda_vision_generate_messages_cpp _barracuda_vision_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(barracuda_vision_gencpp)
add_dependencies(barracuda_vision_gencpp barracuda_vision_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS barracuda_vision_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(barracuda_vision
  "/home/sogo/sogo_ws/src/barracuda_vision/msg/ballInfo.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/barracuda_vision
)
_generate_msg_eus(barracuda_vision
  "/home/sogo/sogo_ws/src/barracuda_vision/msg/ballTravel.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/barracuda_vision
)

### Generating Services

### Generating Module File
_generate_module_eus(barracuda_vision
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/barracuda_vision
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(barracuda_vision_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(barracuda_vision_generate_messages barracuda_vision_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/sogo/sogo_ws/src/barracuda_vision/msg/ballInfo.msg" NAME_WE)
add_dependencies(barracuda_vision_generate_messages_eus _barracuda_vision_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sogo/sogo_ws/src/barracuda_vision/msg/ballTravel.msg" NAME_WE)
add_dependencies(barracuda_vision_generate_messages_eus _barracuda_vision_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(barracuda_vision_geneus)
add_dependencies(barracuda_vision_geneus barracuda_vision_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS barracuda_vision_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(barracuda_vision
  "/home/sogo/sogo_ws/src/barracuda_vision/msg/ballInfo.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/barracuda_vision
)
_generate_msg_lisp(barracuda_vision
  "/home/sogo/sogo_ws/src/barracuda_vision/msg/ballTravel.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/barracuda_vision
)

### Generating Services

### Generating Module File
_generate_module_lisp(barracuda_vision
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/barracuda_vision
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(barracuda_vision_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(barracuda_vision_generate_messages barracuda_vision_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/sogo/sogo_ws/src/barracuda_vision/msg/ballInfo.msg" NAME_WE)
add_dependencies(barracuda_vision_generate_messages_lisp _barracuda_vision_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sogo/sogo_ws/src/barracuda_vision/msg/ballTravel.msg" NAME_WE)
add_dependencies(barracuda_vision_generate_messages_lisp _barracuda_vision_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(barracuda_vision_genlisp)
add_dependencies(barracuda_vision_genlisp barracuda_vision_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS barracuda_vision_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(barracuda_vision
  "/home/sogo/sogo_ws/src/barracuda_vision/msg/ballInfo.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/barracuda_vision
)
_generate_msg_nodejs(barracuda_vision
  "/home/sogo/sogo_ws/src/barracuda_vision/msg/ballTravel.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/barracuda_vision
)

### Generating Services

### Generating Module File
_generate_module_nodejs(barracuda_vision
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/barracuda_vision
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(barracuda_vision_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(barracuda_vision_generate_messages barracuda_vision_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/sogo/sogo_ws/src/barracuda_vision/msg/ballInfo.msg" NAME_WE)
add_dependencies(barracuda_vision_generate_messages_nodejs _barracuda_vision_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sogo/sogo_ws/src/barracuda_vision/msg/ballTravel.msg" NAME_WE)
add_dependencies(barracuda_vision_generate_messages_nodejs _barracuda_vision_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(barracuda_vision_gennodejs)
add_dependencies(barracuda_vision_gennodejs barracuda_vision_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS barracuda_vision_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(barracuda_vision
  "/home/sogo/sogo_ws/src/barracuda_vision/msg/ballInfo.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/barracuda_vision
)
_generate_msg_py(barracuda_vision
  "/home/sogo/sogo_ws/src/barracuda_vision/msg/ballTravel.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/barracuda_vision
)

### Generating Services

### Generating Module File
_generate_module_py(barracuda_vision
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/barracuda_vision
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(barracuda_vision_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(barracuda_vision_generate_messages barracuda_vision_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/sogo/sogo_ws/src/barracuda_vision/msg/ballInfo.msg" NAME_WE)
add_dependencies(barracuda_vision_generate_messages_py _barracuda_vision_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sogo/sogo_ws/src/barracuda_vision/msg/ballTravel.msg" NAME_WE)
add_dependencies(barracuda_vision_generate_messages_py _barracuda_vision_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(barracuda_vision_genpy)
add_dependencies(barracuda_vision_genpy barracuda_vision_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS barracuda_vision_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/barracuda_vision)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/barracuda_vision
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET sensor_msgs_generate_messages_cpp)
  add_dependencies(barracuda_vision_generate_messages_cpp sensor_msgs_generate_messages_cpp)
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(barracuda_vision_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/barracuda_vision)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/barracuda_vision
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET sensor_msgs_generate_messages_eus)
  add_dependencies(barracuda_vision_generate_messages_eus sensor_msgs_generate_messages_eus)
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(barracuda_vision_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/barracuda_vision)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/barracuda_vision
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET sensor_msgs_generate_messages_lisp)
  add_dependencies(barracuda_vision_generate_messages_lisp sensor_msgs_generate_messages_lisp)
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(barracuda_vision_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/barracuda_vision)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/barracuda_vision
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET sensor_msgs_generate_messages_nodejs)
  add_dependencies(barracuda_vision_generate_messages_nodejs sensor_msgs_generate_messages_nodejs)
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(barracuda_vision_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/barracuda_vision)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/barracuda_vision\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/barracuda_vision
    DESTINATION ${genpy_INSTALL_DIR}
    # skip all init files
    PATTERN "__init__.py" EXCLUDE
    PATTERN "__init__.pyc" EXCLUDE
  )
  # install init files which are not in the root folder of the generated code
  string(REGEX REPLACE "([][+.*()^])" "\\\\\\1" ESCAPED_PATH "${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/barracuda_vision")
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/barracuda_vision
    DESTINATION ${genpy_INSTALL_DIR}
    FILES_MATCHING
    REGEX "${ESCAPED_PATH}/.+/__init__.pyc?$"
  )
endif()
if(TARGET sensor_msgs_generate_messages_py)
  add_dependencies(barracuda_vision_generate_messages_py sensor_msgs_generate_messages_py)
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(barracuda_vision_generate_messages_py std_msgs_generate_messages_py)
endif()
