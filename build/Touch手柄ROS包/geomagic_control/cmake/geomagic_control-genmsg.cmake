# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "geomagic_control: 2 messages, 0 services")

set(MSG_I_FLAGS "-Igeomagic_control:/home/smart-robotarm/arm_ws/src/Touch手柄ROS包/geomagic_control/msg;-Igeometry_msgs:/opt/ros/noetic/share/geometry_msgs/cmake/../msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(geomagic_control_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/smart-robotarm/arm_ws/src/Touch手柄ROS包/geomagic_control/msg/DeviceButtonEvent.msg" NAME_WE)
add_custom_target(_geomagic_control_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "geomagic_control" "/home/smart-robotarm/arm_ws/src/Touch手柄ROS包/geomagic_control/msg/DeviceButtonEvent.msg" ""
)

get_filename_component(_filename "/home/smart-robotarm/arm_ws/src/Touch手柄ROS包/geomagic_control/msg/DeviceFeedback.msg" NAME_WE)
add_custom_target(_geomagic_control_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "geomagic_control" "/home/smart-robotarm/arm_ws/src/Touch手柄ROS包/geomagic_control/msg/DeviceFeedback.msg" "geometry_msgs/Vector3"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(geomagic_control
  "/home/smart-robotarm/arm_ws/src/Touch手柄ROS包/geomagic_control/msg/DeviceButtonEvent.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/geomagic_control
)
_generate_msg_cpp(geomagic_control
  "/home/smart-robotarm/arm_ws/src/Touch手柄ROS包/geomagic_control/msg/DeviceFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/geomagic_control
)

### Generating Services

### Generating Module File
_generate_module_cpp(geomagic_control
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/geomagic_control
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(geomagic_control_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(geomagic_control_generate_messages geomagic_control_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/smart-robotarm/arm_ws/src/Touch手柄ROS包/geomagic_control/msg/DeviceButtonEvent.msg" NAME_WE)
add_dependencies(geomagic_control_generate_messages_cpp _geomagic_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/smart-robotarm/arm_ws/src/Touch手柄ROS包/geomagic_control/msg/DeviceFeedback.msg" NAME_WE)
add_dependencies(geomagic_control_generate_messages_cpp _geomagic_control_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(geomagic_control_gencpp)
add_dependencies(geomagic_control_gencpp geomagic_control_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS geomagic_control_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(geomagic_control
  "/home/smart-robotarm/arm_ws/src/Touch手柄ROS包/geomagic_control/msg/DeviceButtonEvent.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/geomagic_control
)
_generate_msg_eus(geomagic_control
  "/home/smart-robotarm/arm_ws/src/Touch手柄ROS包/geomagic_control/msg/DeviceFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/geomagic_control
)

### Generating Services

### Generating Module File
_generate_module_eus(geomagic_control
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/geomagic_control
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(geomagic_control_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(geomagic_control_generate_messages geomagic_control_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/smart-robotarm/arm_ws/src/Touch手柄ROS包/geomagic_control/msg/DeviceButtonEvent.msg" NAME_WE)
add_dependencies(geomagic_control_generate_messages_eus _geomagic_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/smart-robotarm/arm_ws/src/Touch手柄ROS包/geomagic_control/msg/DeviceFeedback.msg" NAME_WE)
add_dependencies(geomagic_control_generate_messages_eus _geomagic_control_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(geomagic_control_geneus)
add_dependencies(geomagic_control_geneus geomagic_control_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS geomagic_control_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(geomagic_control
  "/home/smart-robotarm/arm_ws/src/Touch手柄ROS包/geomagic_control/msg/DeviceButtonEvent.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/geomagic_control
)
_generate_msg_lisp(geomagic_control
  "/home/smart-robotarm/arm_ws/src/Touch手柄ROS包/geomagic_control/msg/DeviceFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/geomagic_control
)

### Generating Services

### Generating Module File
_generate_module_lisp(geomagic_control
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/geomagic_control
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(geomagic_control_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(geomagic_control_generate_messages geomagic_control_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/smart-robotarm/arm_ws/src/Touch手柄ROS包/geomagic_control/msg/DeviceButtonEvent.msg" NAME_WE)
add_dependencies(geomagic_control_generate_messages_lisp _geomagic_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/smart-robotarm/arm_ws/src/Touch手柄ROS包/geomagic_control/msg/DeviceFeedback.msg" NAME_WE)
add_dependencies(geomagic_control_generate_messages_lisp _geomagic_control_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(geomagic_control_genlisp)
add_dependencies(geomagic_control_genlisp geomagic_control_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS geomagic_control_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(geomagic_control
  "/home/smart-robotarm/arm_ws/src/Touch手柄ROS包/geomagic_control/msg/DeviceButtonEvent.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/geomagic_control
)
_generate_msg_nodejs(geomagic_control
  "/home/smart-robotarm/arm_ws/src/Touch手柄ROS包/geomagic_control/msg/DeviceFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/geomagic_control
)

### Generating Services

### Generating Module File
_generate_module_nodejs(geomagic_control
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/geomagic_control
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(geomagic_control_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(geomagic_control_generate_messages geomagic_control_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/smart-robotarm/arm_ws/src/Touch手柄ROS包/geomagic_control/msg/DeviceButtonEvent.msg" NAME_WE)
add_dependencies(geomagic_control_generate_messages_nodejs _geomagic_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/smart-robotarm/arm_ws/src/Touch手柄ROS包/geomagic_control/msg/DeviceFeedback.msg" NAME_WE)
add_dependencies(geomagic_control_generate_messages_nodejs _geomagic_control_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(geomagic_control_gennodejs)
add_dependencies(geomagic_control_gennodejs geomagic_control_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS geomagic_control_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(geomagic_control
  "/home/smart-robotarm/arm_ws/src/Touch手柄ROS包/geomagic_control/msg/DeviceButtonEvent.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/geomagic_control
)
_generate_msg_py(geomagic_control
  "/home/smart-robotarm/arm_ws/src/Touch手柄ROS包/geomagic_control/msg/DeviceFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/geomagic_control
)

### Generating Services

### Generating Module File
_generate_module_py(geomagic_control
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/geomagic_control
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(geomagic_control_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(geomagic_control_generate_messages geomagic_control_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/smart-robotarm/arm_ws/src/Touch手柄ROS包/geomagic_control/msg/DeviceButtonEvent.msg" NAME_WE)
add_dependencies(geomagic_control_generate_messages_py _geomagic_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/smart-robotarm/arm_ws/src/Touch手柄ROS包/geomagic_control/msg/DeviceFeedback.msg" NAME_WE)
add_dependencies(geomagic_control_generate_messages_py _geomagic_control_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(geomagic_control_genpy)
add_dependencies(geomagic_control_genpy geomagic_control_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS geomagic_control_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/geomagic_control)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/geomagic_control
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(geomagic_control_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/geomagic_control)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/geomagic_control
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(geomagic_control_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/geomagic_control)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/geomagic_control
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(geomagic_control_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/geomagic_control)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/geomagic_control
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(geomagic_control_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/geomagic_control)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/geomagic_control\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/geomagic_control
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(geomagic_control_generate_messages_py geometry_msgs_generate_messages_py)
endif()
