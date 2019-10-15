# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "block_pose: 7 messages, 0 services")

set(MSG_I_FLAGS "-Iblock_pose:/home/intelpro/catkin_ws/src/block_pose/msg;-Istd_msgs:/opt/ros/kinetic/share/std_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/kinetic/share/geometry_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(block_pose_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/intelpro/catkin_ws/src/block_pose/msg/RedBlock.msg" NAME_WE)
add_custom_target(_block_pose_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "block_pose" "/home/intelpro/catkin_ws/src/block_pose/msg/RedBlock.msg" "geometry_msgs/Point"
)

get_filename_component(_filename "/home/intelpro/catkin_ws/src/block_pose/msg/PurpleBlock.msg" NAME_WE)
add_custom_target(_block_pose_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "block_pose" "/home/intelpro/catkin_ws/src/block_pose/msg/PurpleBlock.msg" "geometry_msgs/Point"
)

get_filename_component(_filename "/home/intelpro/catkin_ws/src/block_pose/msg/YellowBlock.msg" NAME_WE)
add_custom_target(_block_pose_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "block_pose" "/home/intelpro/catkin_ws/src/block_pose/msg/YellowBlock.msg" "geometry_msgs/Point"
)

get_filename_component(_filename "/home/intelpro/catkin_ws/src/block_pose/msg/OrangeBlock.msg" NAME_WE)
add_custom_target(_block_pose_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "block_pose" "/home/intelpro/catkin_ws/src/block_pose/msg/OrangeBlock.msg" "geometry_msgs/Point"
)

get_filename_component(_filename "/home/intelpro/catkin_ws/src/block_pose/msg/BrownBlock.msg" NAME_WE)
add_custom_target(_block_pose_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "block_pose" "/home/intelpro/catkin_ws/src/block_pose/msg/BrownBlock.msg" "geometry_msgs/Point"
)

get_filename_component(_filename "/home/intelpro/catkin_ws/src/block_pose/msg/GreenBlock.msg" NAME_WE)
add_custom_target(_block_pose_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "block_pose" "/home/intelpro/catkin_ws/src/block_pose/msg/GreenBlock.msg" "geometry_msgs/Point"
)

get_filename_component(_filename "/home/intelpro/catkin_ws/src/block_pose/msg/BlueBlock.msg" NAME_WE)
add_custom_target(_block_pose_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "block_pose" "/home/intelpro/catkin_ws/src/block_pose/msg/BlueBlock.msg" "geometry_msgs/Point"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(block_pose
  "/home/intelpro/catkin_ws/src/block_pose/msg/RedBlock.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/block_pose
)
_generate_msg_cpp(block_pose
  "/home/intelpro/catkin_ws/src/block_pose/msg/PurpleBlock.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/block_pose
)
_generate_msg_cpp(block_pose
  "/home/intelpro/catkin_ws/src/block_pose/msg/YellowBlock.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/block_pose
)
_generate_msg_cpp(block_pose
  "/home/intelpro/catkin_ws/src/block_pose/msg/OrangeBlock.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/block_pose
)
_generate_msg_cpp(block_pose
  "/home/intelpro/catkin_ws/src/block_pose/msg/BrownBlock.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/block_pose
)
_generate_msg_cpp(block_pose
  "/home/intelpro/catkin_ws/src/block_pose/msg/GreenBlock.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/block_pose
)
_generate_msg_cpp(block_pose
  "/home/intelpro/catkin_ws/src/block_pose/msg/BlueBlock.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/block_pose
)

### Generating Services

### Generating Module File
_generate_module_cpp(block_pose
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/block_pose
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(block_pose_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(block_pose_generate_messages block_pose_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/intelpro/catkin_ws/src/block_pose/msg/RedBlock.msg" NAME_WE)
add_dependencies(block_pose_generate_messages_cpp _block_pose_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/intelpro/catkin_ws/src/block_pose/msg/PurpleBlock.msg" NAME_WE)
add_dependencies(block_pose_generate_messages_cpp _block_pose_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/intelpro/catkin_ws/src/block_pose/msg/YellowBlock.msg" NAME_WE)
add_dependencies(block_pose_generate_messages_cpp _block_pose_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/intelpro/catkin_ws/src/block_pose/msg/OrangeBlock.msg" NAME_WE)
add_dependencies(block_pose_generate_messages_cpp _block_pose_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/intelpro/catkin_ws/src/block_pose/msg/BrownBlock.msg" NAME_WE)
add_dependencies(block_pose_generate_messages_cpp _block_pose_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/intelpro/catkin_ws/src/block_pose/msg/GreenBlock.msg" NAME_WE)
add_dependencies(block_pose_generate_messages_cpp _block_pose_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/intelpro/catkin_ws/src/block_pose/msg/BlueBlock.msg" NAME_WE)
add_dependencies(block_pose_generate_messages_cpp _block_pose_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(block_pose_gencpp)
add_dependencies(block_pose_gencpp block_pose_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS block_pose_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(block_pose
  "/home/intelpro/catkin_ws/src/block_pose/msg/RedBlock.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/block_pose
)
_generate_msg_eus(block_pose
  "/home/intelpro/catkin_ws/src/block_pose/msg/PurpleBlock.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/block_pose
)
_generate_msg_eus(block_pose
  "/home/intelpro/catkin_ws/src/block_pose/msg/YellowBlock.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/block_pose
)
_generate_msg_eus(block_pose
  "/home/intelpro/catkin_ws/src/block_pose/msg/OrangeBlock.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/block_pose
)
_generate_msg_eus(block_pose
  "/home/intelpro/catkin_ws/src/block_pose/msg/BrownBlock.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/block_pose
)
_generate_msg_eus(block_pose
  "/home/intelpro/catkin_ws/src/block_pose/msg/GreenBlock.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/block_pose
)
_generate_msg_eus(block_pose
  "/home/intelpro/catkin_ws/src/block_pose/msg/BlueBlock.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/block_pose
)

### Generating Services

### Generating Module File
_generate_module_eus(block_pose
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/block_pose
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(block_pose_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(block_pose_generate_messages block_pose_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/intelpro/catkin_ws/src/block_pose/msg/RedBlock.msg" NAME_WE)
add_dependencies(block_pose_generate_messages_eus _block_pose_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/intelpro/catkin_ws/src/block_pose/msg/PurpleBlock.msg" NAME_WE)
add_dependencies(block_pose_generate_messages_eus _block_pose_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/intelpro/catkin_ws/src/block_pose/msg/YellowBlock.msg" NAME_WE)
add_dependencies(block_pose_generate_messages_eus _block_pose_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/intelpro/catkin_ws/src/block_pose/msg/OrangeBlock.msg" NAME_WE)
add_dependencies(block_pose_generate_messages_eus _block_pose_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/intelpro/catkin_ws/src/block_pose/msg/BrownBlock.msg" NAME_WE)
add_dependencies(block_pose_generate_messages_eus _block_pose_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/intelpro/catkin_ws/src/block_pose/msg/GreenBlock.msg" NAME_WE)
add_dependencies(block_pose_generate_messages_eus _block_pose_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/intelpro/catkin_ws/src/block_pose/msg/BlueBlock.msg" NAME_WE)
add_dependencies(block_pose_generate_messages_eus _block_pose_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(block_pose_geneus)
add_dependencies(block_pose_geneus block_pose_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS block_pose_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(block_pose
  "/home/intelpro/catkin_ws/src/block_pose/msg/RedBlock.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/block_pose
)
_generate_msg_lisp(block_pose
  "/home/intelpro/catkin_ws/src/block_pose/msg/PurpleBlock.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/block_pose
)
_generate_msg_lisp(block_pose
  "/home/intelpro/catkin_ws/src/block_pose/msg/YellowBlock.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/block_pose
)
_generate_msg_lisp(block_pose
  "/home/intelpro/catkin_ws/src/block_pose/msg/OrangeBlock.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/block_pose
)
_generate_msg_lisp(block_pose
  "/home/intelpro/catkin_ws/src/block_pose/msg/BrownBlock.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/block_pose
)
_generate_msg_lisp(block_pose
  "/home/intelpro/catkin_ws/src/block_pose/msg/GreenBlock.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/block_pose
)
_generate_msg_lisp(block_pose
  "/home/intelpro/catkin_ws/src/block_pose/msg/BlueBlock.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/block_pose
)

### Generating Services

### Generating Module File
_generate_module_lisp(block_pose
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/block_pose
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(block_pose_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(block_pose_generate_messages block_pose_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/intelpro/catkin_ws/src/block_pose/msg/RedBlock.msg" NAME_WE)
add_dependencies(block_pose_generate_messages_lisp _block_pose_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/intelpro/catkin_ws/src/block_pose/msg/PurpleBlock.msg" NAME_WE)
add_dependencies(block_pose_generate_messages_lisp _block_pose_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/intelpro/catkin_ws/src/block_pose/msg/YellowBlock.msg" NAME_WE)
add_dependencies(block_pose_generate_messages_lisp _block_pose_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/intelpro/catkin_ws/src/block_pose/msg/OrangeBlock.msg" NAME_WE)
add_dependencies(block_pose_generate_messages_lisp _block_pose_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/intelpro/catkin_ws/src/block_pose/msg/BrownBlock.msg" NAME_WE)
add_dependencies(block_pose_generate_messages_lisp _block_pose_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/intelpro/catkin_ws/src/block_pose/msg/GreenBlock.msg" NAME_WE)
add_dependencies(block_pose_generate_messages_lisp _block_pose_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/intelpro/catkin_ws/src/block_pose/msg/BlueBlock.msg" NAME_WE)
add_dependencies(block_pose_generate_messages_lisp _block_pose_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(block_pose_genlisp)
add_dependencies(block_pose_genlisp block_pose_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS block_pose_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(block_pose
  "/home/intelpro/catkin_ws/src/block_pose/msg/RedBlock.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/block_pose
)
_generate_msg_nodejs(block_pose
  "/home/intelpro/catkin_ws/src/block_pose/msg/PurpleBlock.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/block_pose
)
_generate_msg_nodejs(block_pose
  "/home/intelpro/catkin_ws/src/block_pose/msg/YellowBlock.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/block_pose
)
_generate_msg_nodejs(block_pose
  "/home/intelpro/catkin_ws/src/block_pose/msg/OrangeBlock.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/block_pose
)
_generate_msg_nodejs(block_pose
  "/home/intelpro/catkin_ws/src/block_pose/msg/BrownBlock.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/block_pose
)
_generate_msg_nodejs(block_pose
  "/home/intelpro/catkin_ws/src/block_pose/msg/GreenBlock.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/block_pose
)
_generate_msg_nodejs(block_pose
  "/home/intelpro/catkin_ws/src/block_pose/msg/BlueBlock.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/block_pose
)

### Generating Services

### Generating Module File
_generate_module_nodejs(block_pose
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/block_pose
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(block_pose_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(block_pose_generate_messages block_pose_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/intelpro/catkin_ws/src/block_pose/msg/RedBlock.msg" NAME_WE)
add_dependencies(block_pose_generate_messages_nodejs _block_pose_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/intelpro/catkin_ws/src/block_pose/msg/PurpleBlock.msg" NAME_WE)
add_dependencies(block_pose_generate_messages_nodejs _block_pose_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/intelpro/catkin_ws/src/block_pose/msg/YellowBlock.msg" NAME_WE)
add_dependencies(block_pose_generate_messages_nodejs _block_pose_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/intelpro/catkin_ws/src/block_pose/msg/OrangeBlock.msg" NAME_WE)
add_dependencies(block_pose_generate_messages_nodejs _block_pose_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/intelpro/catkin_ws/src/block_pose/msg/BrownBlock.msg" NAME_WE)
add_dependencies(block_pose_generate_messages_nodejs _block_pose_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/intelpro/catkin_ws/src/block_pose/msg/GreenBlock.msg" NAME_WE)
add_dependencies(block_pose_generate_messages_nodejs _block_pose_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/intelpro/catkin_ws/src/block_pose/msg/BlueBlock.msg" NAME_WE)
add_dependencies(block_pose_generate_messages_nodejs _block_pose_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(block_pose_gennodejs)
add_dependencies(block_pose_gennodejs block_pose_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS block_pose_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(block_pose
  "/home/intelpro/catkin_ws/src/block_pose/msg/RedBlock.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/block_pose
)
_generate_msg_py(block_pose
  "/home/intelpro/catkin_ws/src/block_pose/msg/PurpleBlock.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/block_pose
)
_generate_msg_py(block_pose
  "/home/intelpro/catkin_ws/src/block_pose/msg/YellowBlock.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/block_pose
)
_generate_msg_py(block_pose
  "/home/intelpro/catkin_ws/src/block_pose/msg/OrangeBlock.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/block_pose
)
_generate_msg_py(block_pose
  "/home/intelpro/catkin_ws/src/block_pose/msg/BrownBlock.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/block_pose
)
_generate_msg_py(block_pose
  "/home/intelpro/catkin_ws/src/block_pose/msg/GreenBlock.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/block_pose
)
_generate_msg_py(block_pose
  "/home/intelpro/catkin_ws/src/block_pose/msg/BlueBlock.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/block_pose
)

### Generating Services

### Generating Module File
_generate_module_py(block_pose
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/block_pose
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(block_pose_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(block_pose_generate_messages block_pose_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/intelpro/catkin_ws/src/block_pose/msg/RedBlock.msg" NAME_WE)
add_dependencies(block_pose_generate_messages_py _block_pose_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/intelpro/catkin_ws/src/block_pose/msg/PurpleBlock.msg" NAME_WE)
add_dependencies(block_pose_generate_messages_py _block_pose_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/intelpro/catkin_ws/src/block_pose/msg/YellowBlock.msg" NAME_WE)
add_dependencies(block_pose_generate_messages_py _block_pose_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/intelpro/catkin_ws/src/block_pose/msg/OrangeBlock.msg" NAME_WE)
add_dependencies(block_pose_generate_messages_py _block_pose_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/intelpro/catkin_ws/src/block_pose/msg/BrownBlock.msg" NAME_WE)
add_dependencies(block_pose_generate_messages_py _block_pose_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/intelpro/catkin_ws/src/block_pose/msg/GreenBlock.msg" NAME_WE)
add_dependencies(block_pose_generate_messages_py _block_pose_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/intelpro/catkin_ws/src/block_pose/msg/BlueBlock.msg" NAME_WE)
add_dependencies(block_pose_generate_messages_py _block_pose_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(block_pose_genpy)
add_dependencies(block_pose_genpy block_pose_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS block_pose_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/block_pose)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/block_pose
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(block_pose_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(block_pose_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/block_pose)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/block_pose
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(block_pose_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(block_pose_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/block_pose)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/block_pose
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(block_pose_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(block_pose_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/block_pose)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/block_pose
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(block_pose_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(block_pose_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/block_pose)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/block_pose\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/block_pose
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(block_pose_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(block_pose_generate_messages_py geometry_msgs_generate_messages_py)
endif()
