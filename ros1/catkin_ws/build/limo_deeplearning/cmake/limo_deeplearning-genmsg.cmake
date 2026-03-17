# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "limo_deeplearning: 4 messages, 0 services")

set(MSG_I_FLAGS "-Ilimo_deeplearning:/home/wego/ros1/catkin_ws/src/limo_deeplearning/msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(limo_deeplearning_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/wego/ros1/catkin_ws/src/limo_deeplearning/msg/light.msg" NAME_WE)
add_custom_target(_limo_deeplearning_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "limo_deeplearning" "/home/wego/ros1/catkin_ws/src/limo_deeplearning/msg/light.msg" ""
)

get_filename_component(_filename "/home/wego/ros1/catkin_ws/src/limo_deeplearning/msg/identify.msg" NAME_WE)
add_custom_target(_limo_deeplearning_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "limo_deeplearning" "/home/wego/ros1/catkin_ws/src/limo_deeplearning/msg/identify.msg" ""
)

get_filename_component(_filename "/home/wego/ros1/catkin_ws/src/limo_deeplearning/msg/traffic.msg" NAME_WE)
add_custom_target(_limo_deeplearning_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "limo_deeplearning" "/home/wego/ros1/catkin_ws/src/limo_deeplearning/msg/traffic.msg" ""
)

get_filename_component(_filename "/home/wego/ros1/catkin_ws/src/limo_deeplearning/msg/img.msg" NAME_WE)
add_custom_target(_limo_deeplearning_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "limo_deeplearning" "/home/wego/ros1/catkin_ws/src/limo_deeplearning/msg/img.msg" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(limo_deeplearning
  "/home/wego/ros1/catkin_ws/src/limo_deeplearning/msg/light.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/limo_deeplearning
)
_generate_msg_cpp(limo_deeplearning
  "/home/wego/ros1/catkin_ws/src/limo_deeplearning/msg/identify.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/limo_deeplearning
)
_generate_msg_cpp(limo_deeplearning
  "/home/wego/ros1/catkin_ws/src/limo_deeplearning/msg/traffic.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/limo_deeplearning
)
_generate_msg_cpp(limo_deeplearning
  "/home/wego/ros1/catkin_ws/src/limo_deeplearning/msg/img.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/limo_deeplearning
)

### Generating Services

### Generating Module File
_generate_module_cpp(limo_deeplearning
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/limo_deeplearning
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(limo_deeplearning_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(limo_deeplearning_generate_messages limo_deeplearning_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/wego/ros1/catkin_ws/src/limo_deeplearning/msg/light.msg" NAME_WE)
add_dependencies(limo_deeplearning_generate_messages_cpp _limo_deeplearning_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/wego/ros1/catkin_ws/src/limo_deeplearning/msg/identify.msg" NAME_WE)
add_dependencies(limo_deeplearning_generate_messages_cpp _limo_deeplearning_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/wego/ros1/catkin_ws/src/limo_deeplearning/msg/traffic.msg" NAME_WE)
add_dependencies(limo_deeplearning_generate_messages_cpp _limo_deeplearning_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/wego/ros1/catkin_ws/src/limo_deeplearning/msg/img.msg" NAME_WE)
add_dependencies(limo_deeplearning_generate_messages_cpp _limo_deeplearning_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(limo_deeplearning_gencpp)
add_dependencies(limo_deeplearning_gencpp limo_deeplearning_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS limo_deeplearning_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(limo_deeplearning
  "/home/wego/ros1/catkin_ws/src/limo_deeplearning/msg/light.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/limo_deeplearning
)
_generate_msg_eus(limo_deeplearning
  "/home/wego/ros1/catkin_ws/src/limo_deeplearning/msg/identify.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/limo_deeplearning
)
_generate_msg_eus(limo_deeplearning
  "/home/wego/ros1/catkin_ws/src/limo_deeplearning/msg/traffic.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/limo_deeplearning
)
_generate_msg_eus(limo_deeplearning
  "/home/wego/ros1/catkin_ws/src/limo_deeplearning/msg/img.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/limo_deeplearning
)

### Generating Services

### Generating Module File
_generate_module_eus(limo_deeplearning
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/limo_deeplearning
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(limo_deeplearning_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(limo_deeplearning_generate_messages limo_deeplearning_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/wego/ros1/catkin_ws/src/limo_deeplearning/msg/light.msg" NAME_WE)
add_dependencies(limo_deeplearning_generate_messages_eus _limo_deeplearning_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/wego/ros1/catkin_ws/src/limo_deeplearning/msg/identify.msg" NAME_WE)
add_dependencies(limo_deeplearning_generate_messages_eus _limo_deeplearning_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/wego/ros1/catkin_ws/src/limo_deeplearning/msg/traffic.msg" NAME_WE)
add_dependencies(limo_deeplearning_generate_messages_eus _limo_deeplearning_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/wego/ros1/catkin_ws/src/limo_deeplearning/msg/img.msg" NAME_WE)
add_dependencies(limo_deeplearning_generate_messages_eus _limo_deeplearning_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(limo_deeplearning_geneus)
add_dependencies(limo_deeplearning_geneus limo_deeplearning_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS limo_deeplearning_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(limo_deeplearning
  "/home/wego/ros1/catkin_ws/src/limo_deeplearning/msg/light.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/limo_deeplearning
)
_generate_msg_lisp(limo_deeplearning
  "/home/wego/ros1/catkin_ws/src/limo_deeplearning/msg/identify.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/limo_deeplearning
)
_generate_msg_lisp(limo_deeplearning
  "/home/wego/ros1/catkin_ws/src/limo_deeplearning/msg/traffic.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/limo_deeplearning
)
_generate_msg_lisp(limo_deeplearning
  "/home/wego/ros1/catkin_ws/src/limo_deeplearning/msg/img.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/limo_deeplearning
)

### Generating Services

### Generating Module File
_generate_module_lisp(limo_deeplearning
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/limo_deeplearning
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(limo_deeplearning_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(limo_deeplearning_generate_messages limo_deeplearning_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/wego/ros1/catkin_ws/src/limo_deeplearning/msg/light.msg" NAME_WE)
add_dependencies(limo_deeplearning_generate_messages_lisp _limo_deeplearning_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/wego/ros1/catkin_ws/src/limo_deeplearning/msg/identify.msg" NAME_WE)
add_dependencies(limo_deeplearning_generate_messages_lisp _limo_deeplearning_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/wego/ros1/catkin_ws/src/limo_deeplearning/msg/traffic.msg" NAME_WE)
add_dependencies(limo_deeplearning_generate_messages_lisp _limo_deeplearning_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/wego/ros1/catkin_ws/src/limo_deeplearning/msg/img.msg" NAME_WE)
add_dependencies(limo_deeplearning_generate_messages_lisp _limo_deeplearning_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(limo_deeplearning_genlisp)
add_dependencies(limo_deeplearning_genlisp limo_deeplearning_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS limo_deeplearning_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(limo_deeplearning
  "/home/wego/ros1/catkin_ws/src/limo_deeplearning/msg/light.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/limo_deeplearning
)
_generate_msg_nodejs(limo_deeplearning
  "/home/wego/ros1/catkin_ws/src/limo_deeplearning/msg/identify.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/limo_deeplearning
)
_generate_msg_nodejs(limo_deeplearning
  "/home/wego/ros1/catkin_ws/src/limo_deeplearning/msg/traffic.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/limo_deeplearning
)
_generate_msg_nodejs(limo_deeplearning
  "/home/wego/ros1/catkin_ws/src/limo_deeplearning/msg/img.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/limo_deeplearning
)

### Generating Services

### Generating Module File
_generate_module_nodejs(limo_deeplearning
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/limo_deeplearning
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(limo_deeplearning_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(limo_deeplearning_generate_messages limo_deeplearning_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/wego/ros1/catkin_ws/src/limo_deeplearning/msg/light.msg" NAME_WE)
add_dependencies(limo_deeplearning_generate_messages_nodejs _limo_deeplearning_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/wego/ros1/catkin_ws/src/limo_deeplearning/msg/identify.msg" NAME_WE)
add_dependencies(limo_deeplearning_generate_messages_nodejs _limo_deeplearning_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/wego/ros1/catkin_ws/src/limo_deeplearning/msg/traffic.msg" NAME_WE)
add_dependencies(limo_deeplearning_generate_messages_nodejs _limo_deeplearning_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/wego/ros1/catkin_ws/src/limo_deeplearning/msg/img.msg" NAME_WE)
add_dependencies(limo_deeplearning_generate_messages_nodejs _limo_deeplearning_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(limo_deeplearning_gennodejs)
add_dependencies(limo_deeplearning_gennodejs limo_deeplearning_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS limo_deeplearning_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(limo_deeplearning
  "/home/wego/ros1/catkin_ws/src/limo_deeplearning/msg/light.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/limo_deeplearning
)
_generate_msg_py(limo_deeplearning
  "/home/wego/ros1/catkin_ws/src/limo_deeplearning/msg/identify.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/limo_deeplearning
)
_generate_msg_py(limo_deeplearning
  "/home/wego/ros1/catkin_ws/src/limo_deeplearning/msg/traffic.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/limo_deeplearning
)
_generate_msg_py(limo_deeplearning
  "/home/wego/ros1/catkin_ws/src/limo_deeplearning/msg/img.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/limo_deeplearning
)

### Generating Services

### Generating Module File
_generate_module_py(limo_deeplearning
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/limo_deeplearning
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(limo_deeplearning_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(limo_deeplearning_generate_messages limo_deeplearning_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/wego/ros1/catkin_ws/src/limo_deeplearning/msg/light.msg" NAME_WE)
add_dependencies(limo_deeplearning_generate_messages_py _limo_deeplearning_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/wego/ros1/catkin_ws/src/limo_deeplearning/msg/identify.msg" NAME_WE)
add_dependencies(limo_deeplearning_generate_messages_py _limo_deeplearning_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/wego/ros1/catkin_ws/src/limo_deeplearning/msg/traffic.msg" NAME_WE)
add_dependencies(limo_deeplearning_generate_messages_py _limo_deeplearning_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/wego/ros1/catkin_ws/src/limo_deeplearning/msg/img.msg" NAME_WE)
add_dependencies(limo_deeplearning_generate_messages_py _limo_deeplearning_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(limo_deeplearning_genpy)
add_dependencies(limo_deeplearning_genpy limo_deeplearning_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS limo_deeplearning_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/limo_deeplearning)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/limo_deeplearning
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(limo_deeplearning_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/limo_deeplearning)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/limo_deeplearning
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(limo_deeplearning_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/limo_deeplearning)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/limo_deeplearning
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(limo_deeplearning_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/limo_deeplearning)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/limo_deeplearning
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(limo_deeplearning_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/limo_deeplearning)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/limo_deeplearning\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/limo_deeplearning
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(limo_deeplearning_generate_messages_py std_msgs_generate_messages_py)
endif()
