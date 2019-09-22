# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "matching_algorithm: 2 messages, 0 services")

set(MSG_I_FLAGS "-Imatching_algorithm:/home/intelpro/catkin_ws/src/matching_algorithm/msg;-Istd_msgs:/opt/ros/kinetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(matching_algorithm_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/intelpro/catkin_ws/src/matching_algorithm/msg/Data1d.msg" NAME_WE)
add_custom_target(_matching_algorithm_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "matching_algorithm" "/home/intelpro/catkin_ws/src/matching_algorithm/msg/Data1d.msg" ""
)

get_filename_component(_filename "/home/intelpro/catkin_ws/src/matching_algorithm/msg/Data2d.msg" NAME_WE)
add_custom_target(_matching_algorithm_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "matching_algorithm" "/home/intelpro/catkin_ws/src/matching_algorithm/msg/Data2d.msg" "matching_algorithm/Data1d"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(matching_algorithm
  "/home/intelpro/catkin_ws/src/matching_algorithm/msg/Data1d.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/matching_algorithm
)
_generate_msg_cpp(matching_algorithm
  "/home/intelpro/catkin_ws/src/matching_algorithm/msg/Data2d.msg"
  "${MSG_I_FLAGS}"
  "/home/intelpro/catkin_ws/src/matching_algorithm/msg/Data1d.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/matching_algorithm
)

### Generating Services

### Generating Module File
_generate_module_cpp(matching_algorithm
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/matching_algorithm
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(matching_algorithm_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(matching_algorithm_generate_messages matching_algorithm_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/intelpro/catkin_ws/src/matching_algorithm/msg/Data1d.msg" NAME_WE)
add_dependencies(matching_algorithm_generate_messages_cpp _matching_algorithm_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/intelpro/catkin_ws/src/matching_algorithm/msg/Data2d.msg" NAME_WE)
add_dependencies(matching_algorithm_generate_messages_cpp _matching_algorithm_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(matching_algorithm_gencpp)
add_dependencies(matching_algorithm_gencpp matching_algorithm_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS matching_algorithm_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(matching_algorithm
  "/home/intelpro/catkin_ws/src/matching_algorithm/msg/Data1d.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/matching_algorithm
)
_generate_msg_eus(matching_algorithm
  "/home/intelpro/catkin_ws/src/matching_algorithm/msg/Data2d.msg"
  "${MSG_I_FLAGS}"
  "/home/intelpro/catkin_ws/src/matching_algorithm/msg/Data1d.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/matching_algorithm
)

### Generating Services

### Generating Module File
_generate_module_eus(matching_algorithm
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/matching_algorithm
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(matching_algorithm_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(matching_algorithm_generate_messages matching_algorithm_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/intelpro/catkin_ws/src/matching_algorithm/msg/Data1d.msg" NAME_WE)
add_dependencies(matching_algorithm_generate_messages_eus _matching_algorithm_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/intelpro/catkin_ws/src/matching_algorithm/msg/Data2d.msg" NAME_WE)
add_dependencies(matching_algorithm_generate_messages_eus _matching_algorithm_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(matching_algorithm_geneus)
add_dependencies(matching_algorithm_geneus matching_algorithm_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS matching_algorithm_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(matching_algorithm
  "/home/intelpro/catkin_ws/src/matching_algorithm/msg/Data1d.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/matching_algorithm
)
_generate_msg_lisp(matching_algorithm
  "/home/intelpro/catkin_ws/src/matching_algorithm/msg/Data2d.msg"
  "${MSG_I_FLAGS}"
  "/home/intelpro/catkin_ws/src/matching_algorithm/msg/Data1d.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/matching_algorithm
)

### Generating Services

### Generating Module File
_generate_module_lisp(matching_algorithm
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/matching_algorithm
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(matching_algorithm_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(matching_algorithm_generate_messages matching_algorithm_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/intelpro/catkin_ws/src/matching_algorithm/msg/Data1d.msg" NAME_WE)
add_dependencies(matching_algorithm_generate_messages_lisp _matching_algorithm_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/intelpro/catkin_ws/src/matching_algorithm/msg/Data2d.msg" NAME_WE)
add_dependencies(matching_algorithm_generate_messages_lisp _matching_algorithm_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(matching_algorithm_genlisp)
add_dependencies(matching_algorithm_genlisp matching_algorithm_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS matching_algorithm_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(matching_algorithm
  "/home/intelpro/catkin_ws/src/matching_algorithm/msg/Data1d.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/matching_algorithm
)
_generate_msg_nodejs(matching_algorithm
  "/home/intelpro/catkin_ws/src/matching_algorithm/msg/Data2d.msg"
  "${MSG_I_FLAGS}"
  "/home/intelpro/catkin_ws/src/matching_algorithm/msg/Data1d.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/matching_algorithm
)

### Generating Services

### Generating Module File
_generate_module_nodejs(matching_algorithm
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/matching_algorithm
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(matching_algorithm_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(matching_algorithm_generate_messages matching_algorithm_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/intelpro/catkin_ws/src/matching_algorithm/msg/Data1d.msg" NAME_WE)
add_dependencies(matching_algorithm_generate_messages_nodejs _matching_algorithm_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/intelpro/catkin_ws/src/matching_algorithm/msg/Data2d.msg" NAME_WE)
add_dependencies(matching_algorithm_generate_messages_nodejs _matching_algorithm_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(matching_algorithm_gennodejs)
add_dependencies(matching_algorithm_gennodejs matching_algorithm_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS matching_algorithm_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(matching_algorithm
  "/home/intelpro/catkin_ws/src/matching_algorithm/msg/Data1d.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/matching_algorithm
)
_generate_msg_py(matching_algorithm
  "/home/intelpro/catkin_ws/src/matching_algorithm/msg/Data2d.msg"
  "${MSG_I_FLAGS}"
  "/home/intelpro/catkin_ws/src/matching_algorithm/msg/Data1d.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/matching_algorithm
)

### Generating Services

### Generating Module File
_generate_module_py(matching_algorithm
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/matching_algorithm
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(matching_algorithm_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(matching_algorithm_generate_messages matching_algorithm_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/intelpro/catkin_ws/src/matching_algorithm/msg/Data1d.msg" NAME_WE)
add_dependencies(matching_algorithm_generate_messages_py _matching_algorithm_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/intelpro/catkin_ws/src/matching_algorithm/msg/Data2d.msg" NAME_WE)
add_dependencies(matching_algorithm_generate_messages_py _matching_algorithm_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(matching_algorithm_genpy)
add_dependencies(matching_algorithm_genpy matching_algorithm_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS matching_algorithm_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/matching_algorithm)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/matching_algorithm
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(matching_algorithm_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/matching_algorithm)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/matching_algorithm
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(matching_algorithm_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/matching_algorithm)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/matching_algorithm
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(matching_algorithm_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/matching_algorithm)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/matching_algorithm
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(matching_algorithm_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/matching_algorithm)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/matching_algorithm\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/matching_algorithm
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(matching_algorithm_generate_messages_py std_msgs_generate_messages_py)
endif()
