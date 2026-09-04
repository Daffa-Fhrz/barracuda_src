# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "barracuda_kinematic: 1 messages, 0 services")

set(MSG_I_FLAGS "-Ibarracuda_kinematic:/home/sogo/sogo_ws/src/barracuda_kinematic/msg;-Iactionlib_msgs:/opt/ros/noetic/share/actionlib_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/noetic/share/geometry_msgs/cmake/../msg;-Inav_msgs:/opt/ros/noetic/share/nav_msgs/cmake/../msg;-Isensor_msgs:/opt/ros/noetic/share/sensor_msgs/cmake/../msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(barracuda_kinematic_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/sogo/sogo_ws/src/barracuda_kinematic/msg/robotSpeed.msg" NAME_WE)
add_custom_target(_barracuda_kinematic_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "barracuda_kinematic" "/home/sogo/sogo_ws/src/barracuda_kinematic/msg/robotSpeed.msg" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(barracuda_kinematic
  "/home/sogo/sogo_ws/src/barracuda_kinematic/msg/robotSpeed.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/barracuda_kinematic
)

### Generating Services

### Generating Module File
_generate_module_cpp(barracuda_kinematic
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/barracuda_kinematic
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(barracuda_kinematic_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(barracuda_kinematic_generate_messages barracuda_kinematic_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/sogo/sogo_ws/src/barracuda_kinematic/msg/robotSpeed.msg" NAME_WE)
add_dependencies(barracuda_kinematic_generate_messages_cpp _barracuda_kinematic_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(barracuda_kinematic_gencpp)
add_dependencies(barracuda_kinematic_gencpp barracuda_kinematic_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS barracuda_kinematic_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(barracuda_kinematic
  "/home/sogo/sogo_ws/src/barracuda_kinematic/msg/robotSpeed.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/barracuda_kinematic
)

### Generating Services

### Generating Module File
_generate_module_eus(barracuda_kinematic
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/barracuda_kinematic
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(barracuda_kinematic_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(barracuda_kinematic_generate_messages barracuda_kinematic_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/sogo/sogo_ws/src/barracuda_kinematic/msg/robotSpeed.msg" NAME_WE)
add_dependencies(barracuda_kinematic_generate_messages_eus _barracuda_kinematic_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(barracuda_kinematic_geneus)
add_dependencies(barracuda_kinematic_geneus barracuda_kinematic_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS barracuda_kinematic_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(barracuda_kinematic
  "/home/sogo/sogo_ws/src/barracuda_kinematic/msg/robotSpeed.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/barracuda_kinematic
)

### Generating Services

### Generating Module File
_generate_module_lisp(barracuda_kinematic
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/barracuda_kinematic
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(barracuda_kinematic_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(barracuda_kinematic_generate_messages barracuda_kinematic_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/sogo/sogo_ws/src/barracuda_kinematic/msg/robotSpeed.msg" NAME_WE)
add_dependencies(barracuda_kinematic_generate_messages_lisp _barracuda_kinematic_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(barracuda_kinematic_genlisp)
add_dependencies(barracuda_kinematic_genlisp barracuda_kinematic_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS barracuda_kinematic_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(barracuda_kinematic
  "/home/sogo/sogo_ws/src/barracuda_kinematic/msg/robotSpeed.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/barracuda_kinematic
)

### Generating Services

### Generating Module File
_generate_module_nodejs(barracuda_kinematic
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/barracuda_kinematic
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(barracuda_kinematic_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(barracuda_kinematic_generate_messages barracuda_kinematic_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/sogo/sogo_ws/src/barracuda_kinematic/msg/robotSpeed.msg" NAME_WE)
add_dependencies(barracuda_kinematic_generate_messages_nodejs _barracuda_kinematic_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(barracuda_kinematic_gennodejs)
add_dependencies(barracuda_kinematic_gennodejs barracuda_kinematic_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS barracuda_kinematic_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(barracuda_kinematic
  "/home/sogo/sogo_ws/src/barracuda_kinematic/msg/robotSpeed.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/barracuda_kinematic
)

### Generating Services

### Generating Module File
_generate_module_py(barracuda_kinematic
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/barracuda_kinematic
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(barracuda_kinematic_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(barracuda_kinematic_generate_messages barracuda_kinematic_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/sogo/sogo_ws/src/barracuda_kinematic/msg/robotSpeed.msg" NAME_WE)
add_dependencies(barracuda_kinematic_generate_messages_py _barracuda_kinematic_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(barracuda_kinematic_genpy)
add_dependencies(barracuda_kinematic_genpy barracuda_kinematic_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS barracuda_kinematic_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/barracuda_kinematic)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/barracuda_kinematic
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET actionlib_msgs_generate_messages_cpp)
  add_dependencies(barracuda_kinematic_generate_messages_cpp actionlib_msgs_generate_messages_cpp)
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(barracuda_kinematic_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()
if(TARGET nav_msgs_generate_messages_cpp)
  add_dependencies(barracuda_kinematic_generate_messages_cpp nav_msgs_generate_messages_cpp)
endif()
if(TARGET sensor_msgs_generate_messages_cpp)
  add_dependencies(barracuda_kinematic_generate_messages_cpp sensor_msgs_generate_messages_cpp)
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(barracuda_kinematic_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/barracuda_kinematic)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/barracuda_kinematic
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET actionlib_msgs_generate_messages_eus)
  add_dependencies(barracuda_kinematic_generate_messages_eus actionlib_msgs_generate_messages_eus)
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(barracuda_kinematic_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()
if(TARGET nav_msgs_generate_messages_eus)
  add_dependencies(barracuda_kinematic_generate_messages_eus nav_msgs_generate_messages_eus)
endif()
if(TARGET sensor_msgs_generate_messages_eus)
  add_dependencies(barracuda_kinematic_generate_messages_eus sensor_msgs_generate_messages_eus)
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(barracuda_kinematic_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/barracuda_kinematic)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/barracuda_kinematic
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET actionlib_msgs_generate_messages_lisp)
  add_dependencies(barracuda_kinematic_generate_messages_lisp actionlib_msgs_generate_messages_lisp)
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(barracuda_kinematic_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()
if(TARGET nav_msgs_generate_messages_lisp)
  add_dependencies(barracuda_kinematic_generate_messages_lisp nav_msgs_generate_messages_lisp)
endif()
if(TARGET sensor_msgs_generate_messages_lisp)
  add_dependencies(barracuda_kinematic_generate_messages_lisp sensor_msgs_generate_messages_lisp)
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(barracuda_kinematic_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/barracuda_kinematic)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/barracuda_kinematic
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET actionlib_msgs_generate_messages_nodejs)
  add_dependencies(barracuda_kinematic_generate_messages_nodejs actionlib_msgs_generate_messages_nodejs)
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(barracuda_kinematic_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()
if(TARGET nav_msgs_generate_messages_nodejs)
  add_dependencies(barracuda_kinematic_generate_messages_nodejs nav_msgs_generate_messages_nodejs)
endif()
if(TARGET sensor_msgs_generate_messages_nodejs)
  add_dependencies(barracuda_kinematic_generate_messages_nodejs sensor_msgs_generate_messages_nodejs)
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(barracuda_kinematic_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/barracuda_kinematic)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/barracuda_kinematic\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/barracuda_kinematic
    DESTINATION ${genpy_INSTALL_DIR}
    # skip all init files
    PATTERN "__init__.py" EXCLUDE
    PATTERN "__init__.pyc" EXCLUDE
  )
  # install init files which are not in the root folder of the generated code
  string(REGEX REPLACE "([][+.*()^])" "\\\\\\1" ESCAPED_PATH "${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/barracuda_kinematic")
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/barracuda_kinematic
    DESTINATION ${genpy_INSTALL_DIR}
    FILES_MATCHING
    REGEX "${ESCAPED_PATH}/.+/__init__.pyc?$"
  )
endif()
if(TARGET actionlib_msgs_generate_messages_py)
  add_dependencies(barracuda_kinematic_generate_messages_py actionlib_msgs_generate_messages_py)
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(barracuda_kinematic_generate_messages_py geometry_msgs_generate_messages_py)
endif()
if(TARGET nav_msgs_generate_messages_py)
  add_dependencies(barracuda_kinematic_generate_messages_py nav_msgs_generate_messages_py)
endif()
if(TARGET sensor_msgs_generate_messages_py)
  add_dependencies(barracuda_kinematic_generate_messages_py sensor_msgs_generate_messages_py)
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(barracuda_kinematic_generate_messages_py std_msgs_generate_messages_py)
endif()
