# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "cc_fabmap: 4 messages, 0 services")

set(MSG_I_FLAGS "-Icc_fabmap:/home/leo/new_cat/src/cc_fabmap/msg;-Isensor_msgs:/opt/ros/indigo/share/sensor_msgs/cmake/../msg;-Istd_msgs:/opt/ros/indigo/share/std_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/indigo/share/geometry_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(genlisp REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(cc_fabmap_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/leo/new_cat/src/cc_fabmap/msg/keyframeGraphMsgStamped.msg" NAME_WE)
add_custom_target(_cc_fabmap_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "cc_fabmap" "/home/leo/new_cat/src/cc_fabmap/msg/keyframeGraphMsgStamped.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/leo/new_cat/src/cc_fabmap/msg/keyframeGraphMsg.msg" NAME_WE)
add_custom_target(_cc_fabmap_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "cc_fabmap" "/home/leo/new_cat/src/cc_fabmap/msg/keyframeGraphMsg.msg" ""
)

get_filename_component(_filename "/home/leo/new_cat/src/cc_fabmap/msg/keyframeMsgStamped.msg" NAME_WE)
add_custom_target(_cc_fabmap_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "cc_fabmap" "/home/leo/new_cat/src/cc_fabmap/msg/keyframeMsgStamped.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/leo/new_cat/src/cc_fabmap/msg/keyframeMsg.msg" NAME_WE)
add_custom_target(_cc_fabmap_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "cc_fabmap" "/home/leo/new_cat/src/cc_fabmap/msg/keyframeMsg.msg" ""
)

#
#  langs = gencpp;genlisp;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(cc_fabmap
  "/home/leo/new_cat/src/cc_fabmap/msg/keyframeGraphMsgStamped.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/cc_fabmap
)
_generate_msg_cpp(cc_fabmap
  "/home/leo/new_cat/src/cc_fabmap/msg/keyframeGraphMsg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/cc_fabmap
)
_generate_msg_cpp(cc_fabmap
  "/home/leo/new_cat/src/cc_fabmap/msg/keyframeMsgStamped.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/cc_fabmap
)
_generate_msg_cpp(cc_fabmap
  "/home/leo/new_cat/src/cc_fabmap/msg/keyframeMsg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/cc_fabmap
)

### Generating Services

### Generating Module File
_generate_module_cpp(cc_fabmap
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/cc_fabmap
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(cc_fabmap_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(cc_fabmap_generate_messages cc_fabmap_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/leo/new_cat/src/cc_fabmap/msg/keyframeGraphMsgStamped.msg" NAME_WE)
add_dependencies(cc_fabmap_generate_messages_cpp _cc_fabmap_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/leo/new_cat/src/cc_fabmap/msg/keyframeGraphMsg.msg" NAME_WE)
add_dependencies(cc_fabmap_generate_messages_cpp _cc_fabmap_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/leo/new_cat/src/cc_fabmap/msg/keyframeMsgStamped.msg" NAME_WE)
add_dependencies(cc_fabmap_generate_messages_cpp _cc_fabmap_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/leo/new_cat/src/cc_fabmap/msg/keyframeMsg.msg" NAME_WE)
add_dependencies(cc_fabmap_generate_messages_cpp _cc_fabmap_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(cc_fabmap_gencpp)
add_dependencies(cc_fabmap_gencpp cc_fabmap_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS cc_fabmap_generate_messages_cpp)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(cc_fabmap
  "/home/leo/new_cat/src/cc_fabmap/msg/keyframeGraphMsgStamped.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/cc_fabmap
)
_generate_msg_lisp(cc_fabmap
  "/home/leo/new_cat/src/cc_fabmap/msg/keyframeGraphMsg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/cc_fabmap
)
_generate_msg_lisp(cc_fabmap
  "/home/leo/new_cat/src/cc_fabmap/msg/keyframeMsgStamped.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/cc_fabmap
)
_generate_msg_lisp(cc_fabmap
  "/home/leo/new_cat/src/cc_fabmap/msg/keyframeMsg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/cc_fabmap
)

### Generating Services

### Generating Module File
_generate_module_lisp(cc_fabmap
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/cc_fabmap
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(cc_fabmap_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(cc_fabmap_generate_messages cc_fabmap_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/leo/new_cat/src/cc_fabmap/msg/keyframeGraphMsgStamped.msg" NAME_WE)
add_dependencies(cc_fabmap_generate_messages_lisp _cc_fabmap_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/leo/new_cat/src/cc_fabmap/msg/keyframeGraphMsg.msg" NAME_WE)
add_dependencies(cc_fabmap_generate_messages_lisp _cc_fabmap_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/leo/new_cat/src/cc_fabmap/msg/keyframeMsgStamped.msg" NAME_WE)
add_dependencies(cc_fabmap_generate_messages_lisp _cc_fabmap_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/leo/new_cat/src/cc_fabmap/msg/keyframeMsg.msg" NAME_WE)
add_dependencies(cc_fabmap_generate_messages_lisp _cc_fabmap_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(cc_fabmap_genlisp)
add_dependencies(cc_fabmap_genlisp cc_fabmap_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS cc_fabmap_generate_messages_lisp)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(cc_fabmap
  "/home/leo/new_cat/src/cc_fabmap/msg/keyframeGraphMsgStamped.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/cc_fabmap
)
_generate_msg_py(cc_fabmap
  "/home/leo/new_cat/src/cc_fabmap/msg/keyframeGraphMsg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/cc_fabmap
)
_generate_msg_py(cc_fabmap
  "/home/leo/new_cat/src/cc_fabmap/msg/keyframeMsgStamped.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/cc_fabmap
)
_generate_msg_py(cc_fabmap
  "/home/leo/new_cat/src/cc_fabmap/msg/keyframeMsg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/cc_fabmap
)

### Generating Services

### Generating Module File
_generate_module_py(cc_fabmap
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/cc_fabmap
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(cc_fabmap_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(cc_fabmap_generate_messages cc_fabmap_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/leo/new_cat/src/cc_fabmap/msg/keyframeGraphMsgStamped.msg" NAME_WE)
add_dependencies(cc_fabmap_generate_messages_py _cc_fabmap_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/leo/new_cat/src/cc_fabmap/msg/keyframeGraphMsg.msg" NAME_WE)
add_dependencies(cc_fabmap_generate_messages_py _cc_fabmap_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/leo/new_cat/src/cc_fabmap/msg/keyframeMsgStamped.msg" NAME_WE)
add_dependencies(cc_fabmap_generate_messages_py _cc_fabmap_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/leo/new_cat/src/cc_fabmap/msg/keyframeMsg.msg" NAME_WE)
add_dependencies(cc_fabmap_generate_messages_py _cc_fabmap_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(cc_fabmap_genpy)
add_dependencies(cc_fabmap_genpy cc_fabmap_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS cc_fabmap_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/cc_fabmap)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/cc_fabmap
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
add_dependencies(cc_fabmap_generate_messages_cpp sensor_msgs_generate_messages_cpp)
add_dependencies(cc_fabmap_generate_messages_cpp std_msgs_generate_messages_cpp)
add_dependencies(cc_fabmap_generate_messages_cpp geometry_msgs_generate_messages_cpp)

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/cc_fabmap)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/cc_fabmap
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
add_dependencies(cc_fabmap_generate_messages_lisp sensor_msgs_generate_messages_lisp)
add_dependencies(cc_fabmap_generate_messages_lisp std_msgs_generate_messages_lisp)
add_dependencies(cc_fabmap_generate_messages_lisp geometry_msgs_generate_messages_lisp)

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/cc_fabmap)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/cc_fabmap\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/cc_fabmap
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
add_dependencies(cc_fabmap_generate_messages_py sensor_msgs_generate_messages_py)
add_dependencies(cc_fabmap_generate_messages_py std_msgs_generate_messages_py)
add_dependencies(cc_fabmap_generate_messages_py geometry_msgs_generate_messages_py)
