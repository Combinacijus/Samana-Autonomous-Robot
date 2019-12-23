# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "samana_msgs: 9 messages, 0 services")

set(MSG_I_FLAGS "-Isamana_msgs:/home/combinacijus/Documents/SamanaAutonomousRobot/ROS/samana_ws/src/samana_msgs/msg;-Istd_msgs:/opt/ros/melodic/share/std_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/melodic/share/geometry_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(samana_msgs_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/combinacijus/Documents/SamanaAutonomousRobot/ROS/samana_ws/src/samana_msgs/msg/Vector4_32.msg" NAME_WE)
add_custom_target(_samana_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "samana_msgs" "/home/combinacijus/Documents/SamanaAutonomousRobot/ROS/samana_ws/src/samana_msgs/msg/Vector4_32.msg" ""
)

get_filename_component(_filename "/home/combinacijus/Documents/SamanaAutonomousRobot/ROS/samana_ws/src/samana_msgs/msg/ArmCmd.msg" NAME_WE)
add_custom_target(_samana_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "samana_msgs" "/home/combinacijus/Documents/SamanaAutonomousRobot/ROS/samana_ws/src/samana_msgs/msg/ArmCmd.msg" ""
)

get_filename_component(_filename "/home/combinacijus/Documents/SamanaAutonomousRobot/ROS/samana_ws/src/samana_msgs/msg/Vector3_32.msg" NAME_WE)
add_custom_target(_samana_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "samana_msgs" "/home/combinacijus/Documents/SamanaAutonomousRobot/ROS/samana_ws/src/samana_msgs/msg/Vector3_32.msg" ""
)

get_filename_component(_filename "/home/combinacijus/Documents/SamanaAutonomousRobot/ROS/samana_ws/src/samana_msgs/msg/Bump.msg" NAME_WE)
add_custom_target(_samana_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "samana_msgs" "/home/combinacijus/Documents/SamanaAutonomousRobot/ROS/samana_ws/src/samana_msgs/msg/Bump.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/combinacijus/Documents/SamanaAutonomousRobot/ROS/samana_ws/src/samana_msgs/msg/ImuSmall.msg" NAME_WE)
add_custom_target(_samana_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "samana_msgs" "/home/combinacijus/Documents/SamanaAutonomousRobot/ROS/samana_ws/src/samana_msgs/msg/ImuSmall.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/combinacijus/Documents/SamanaAutonomousRobot/ROS/samana_ws/src/samana_msgs/msg/ArmData.msg" NAME_WE)
add_custom_target(_samana_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "samana_msgs" "/home/combinacijus/Documents/SamanaAutonomousRobot/ROS/samana_ws/src/samana_msgs/msg/ArmData.msg" ""
)

get_filename_component(_filename "/home/combinacijus/Documents/SamanaAutonomousRobot/ROS/samana_ws/src/samana_msgs/msg/Teleop.msg" NAME_WE)
add_custom_target(_samana_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "samana_msgs" "/home/combinacijus/Documents/SamanaAutonomousRobot/ROS/samana_ws/src/samana_msgs/msg/Teleop.msg" ""
)

get_filename_component(_filename "/home/combinacijus/Documents/SamanaAutonomousRobot/ROS/samana_ws/src/samana_msgs/msg/Int16Array.msg" NAME_WE)
add_custom_target(_samana_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "samana_msgs" "/home/combinacijus/Documents/SamanaAutonomousRobot/ROS/samana_ws/src/samana_msgs/msg/Int16Array.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/combinacijus/Documents/SamanaAutonomousRobot/ROS/samana_ws/src/samana_msgs/msg/ImuCalib.msg" NAME_WE)
add_custom_target(_samana_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "samana_msgs" "/home/combinacijus/Documents/SamanaAutonomousRobot/ROS/samana_ws/src/samana_msgs/msg/ImuCalib.msg" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(samana_msgs
  "/home/combinacijus/Documents/SamanaAutonomousRobot/ROS/samana_ws/src/samana_msgs/msg/Vector4_32.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/samana_msgs
)
_generate_msg_cpp(samana_msgs
  "/home/combinacijus/Documents/SamanaAutonomousRobot/ROS/samana_ws/src/samana_msgs/msg/ArmCmd.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/samana_msgs
)
_generate_msg_cpp(samana_msgs
  "/home/combinacijus/Documents/SamanaAutonomousRobot/ROS/samana_ws/src/samana_msgs/msg/Vector3_32.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/samana_msgs
)
_generate_msg_cpp(samana_msgs
  "/home/combinacijus/Documents/SamanaAutonomousRobot/ROS/samana_ws/src/samana_msgs/msg/Bump.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/samana_msgs
)
_generate_msg_cpp(samana_msgs
  "/home/combinacijus/Documents/SamanaAutonomousRobot/ROS/samana_ws/src/samana_msgs/msg/ImuSmall.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/samana_msgs
)
_generate_msg_cpp(samana_msgs
  "/home/combinacijus/Documents/SamanaAutonomousRobot/ROS/samana_ws/src/samana_msgs/msg/ArmData.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/samana_msgs
)
_generate_msg_cpp(samana_msgs
  "/home/combinacijus/Documents/SamanaAutonomousRobot/ROS/samana_ws/src/samana_msgs/msg/Teleop.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/samana_msgs
)
_generate_msg_cpp(samana_msgs
  "/home/combinacijus/Documents/SamanaAutonomousRobot/ROS/samana_ws/src/samana_msgs/msg/Int16Array.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/samana_msgs
)
_generate_msg_cpp(samana_msgs
  "/home/combinacijus/Documents/SamanaAutonomousRobot/ROS/samana_ws/src/samana_msgs/msg/ImuCalib.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/samana_msgs
)

### Generating Services

### Generating Module File
_generate_module_cpp(samana_msgs
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/samana_msgs
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(samana_msgs_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(samana_msgs_generate_messages samana_msgs_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/combinacijus/Documents/SamanaAutonomousRobot/ROS/samana_ws/src/samana_msgs/msg/Vector4_32.msg" NAME_WE)
add_dependencies(samana_msgs_generate_messages_cpp _samana_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/combinacijus/Documents/SamanaAutonomousRobot/ROS/samana_ws/src/samana_msgs/msg/ArmCmd.msg" NAME_WE)
add_dependencies(samana_msgs_generate_messages_cpp _samana_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/combinacijus/Documents/SamanaAutonomousRobot/ROS/samana_ws/src/samana_msgs/msg/Vector3_32.msg" NAME_WE)
add_dependencies(samana_msgs_generate_messages_cpp _samana_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/combinacijus/Documents/SamanaAutonomousRobot/ROS/samana_ws/src/samana_msgs/msg/Bump.msg" NAME_WE)
add_dependencies(samana_msgs_generate_messages_cpp _samana_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/combinacijus/Documents/SamanaAutonomousRobot/ROS/samana_ws/src/samana_msgs/msg/ImuSmall.msg" NAME_WE)
add_dependencies(samana_msgs_generate_messages_cpp _samana_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/combinacijus/Documents/SamanaAutonomousRobot/ROS/samana_ws/src/samana_msgs/msg/ArmData.msg" NAME_WE)
add_dependencies(samana_msgs_generate_messages_cpp _samana_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/combinacijus/Documents/SamanaAutonomousRobot/ROS/samana_ws/src/samana_msgs/msg/Teleop.msg" NAME_WE)
add_dependencies(samana_msgs_generate_messages_cpp _samana_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/combinacijus/Documents/SamanaAutonomousRobot/ROS/samana_ws/src/samana_msgs/msg/Int16Array.msg" NAME_WE)
add_dependencies(samana_msgs_generate_messages_cpp _samana_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/combinacijus/Documents/SamanaAutonomousRobot/ROS/samana_ws/src/samana_msgs/msg/ImuCalib.msg" NAME_WE)
add_dependencies(samana_msgs_generate_messages_cpp _samana_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(samana_msgs_gencpp)
add_dependencies(samana_msgs_gencpp samana_msgs_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS samana_msgs_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(samana_msgs
  "/home/combinacijus/Documents/SamanaAutonomousRobot/ROS/samana_ws/src/samana_msgs/msg/Vector4_32.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/samana_msgs
)
_generate_msg_eus(samana_msgs
  "/home/combinacijus/Documents/SamanaAutonomousRobot/ROS/samana_ws/src/samana_msgs/msg/ArmCmd.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/samana_msgs
)
_generate_msg_eus(samana_msgs
  "/home/combinacijus/Documents/SamanaAutonomousRobot/ROS/samana_ws/src/samana_msgs/msg/Vector3_32.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/samana_msgs
)
_generate_msg_eus(samana_msgs
  "/home/combinacijus/Documents/SamanaAutonomousRobot/ROS/samana_ws/src/samana_msgs/msg/Bump.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/samana_msgs
)
_generate_msg_eus(samana_msgs
  "/home/combinacijus/Documents/SamanaAutonomousRobot/ROS/samana_ws/src/samana_msgs/msg/ImuSmall.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/samana_msgs
)
_generate_msg_eus(samana_msgs
  "/home/combinacijus/Documents/SamanaAutonomousRobot/ROS/samana_ws/src/samana_msgs/msg/ArmData.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/samana_msgs
)
_generate_msg_eus(samana_msgs
  "/home/combinacijus/Documents/SamanaAutonomousRobot/ROS/samana_ws/src/samana_msgs/msg/Teleop.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/samana_msgs
)
_generate_msg_eus(samana_msgs
  "/home/combinacijus/Documents/SamanaAutonomousRobot/ROS/samana_ws/src/samana_msgs/msg/Int16Array.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/samana_msgs
)
_generate_msg_eus(samana_msgs
  "/home/combinacijus/Documents/SamanaAutonomousRobot/ROS/samana_ws/src/samana_msgs/msg/ImuCalib.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/samana_msgs
)

### Generating Services

### Generating Module File
_generate_module_eus(samana_msgs
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/samana_msgs
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(samana_msgs_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(samana_msgs_generate_messages samana_msgs_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/combinacijus/Documents/SamanaAutonomousRobot/ROS/samana_ws/src/samana_msgs/msg/Vector4_32.msg" NAME_WE)
add_dependencies(samana_msgs_generate_messages_eus _samana_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/combinacijus/Documents/SamanaAutonomousRobot/ROS/samana_ws/src/samana_msgs/msg/ArmCmd.msg" NAME_WE)
add_dependencies(samana_msgs_generate_messages_eus _samana_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/combinacijus/Documents/SamanaAutonomousRobot/ROS/samana_ws/src/samana_msgs/msg/Vector3_32.msg" NAME_WE)
add_dependencies(samana_msgs_generate_messages_eus _samana_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/combinacijus/Documents/SamanaAutonomousRobot/ROS/samana_ws/src/samana_msgs/msg/Bump.msg" NAME_WE)
add_dependencies(samana_msgs_generate_messages_eus _samana_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/combinacijus/Documents/SamanaAutonomousRobot/ROS/samana_ws/src/samana_msgs/msg/ImuSmall.msg" NAME_WE)
add_dependencies(samana_msgs_generate_messages_eus _samana_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/combinacijus/Documents/SamanaAutonomousRobot/ROS/samana_ws/src/samana_msgs/msg/ArmData.msg" NAME_WE)
add_dependencies(samana_msgs_generate_messages_eus _samana_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/combinacijus/Documents/SamanaAutonomousRobot/ROS/samana_ws/src/samana_msgs/msg/Teleop.msg" NAME_WE)
add_dependencies(samana_msgs_generate_messages_eus _samana_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/combinacijus/Documents/SamanaAutonomousRobot/ROS/samana_ws/src/samana_msgs/msg/Int16Array.msg" NAME_WE)
add_dependencies(samana_msgs_generate_messages_eus _samana_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/combinacijus/Documents/SamanaAutonomousRobot/ROS/samana_ws/src/samana_msgs/msg/ImuCalib.msg" NAME_WE)
add_dependencies(samana_msgs_generate_messages_eus _samana_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(samana_msgs_geneus)
add_dependencies(samana_msgs_geneus samana_msgs_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS samana_msgs_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(samana_msgs
  "/home/combinacijus/Documents/SamanaAutonomousRobot/ROS/samana_ws/src/samana_msgs/msg/Vector4_32.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/samana_msgs
)
_generate_msg_lisp(samana_msgs
  "/home/combinacijus/Documents/SamanaAutonomousRobot/ROS/samana_ws/src/samana_msgs/msg/ArmCmd.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/samana_msgs
)
_generate_msg_lisp(samana_msgs
  "/home/combinacijus/Documents/SamanaAutonomousRobot/ROS/samana_ws/src/samana_msgs/msg/Vector3_32.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/samana_msgs
)
_generate_msg_lisp(samana_msgs
  "/home/combinacijus/Documents/SamanaAutonomousRobot/ROS/samana_ws/src/samana_msgs/msg/Bump.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/samana_msgs
)
_generate_msg_lisp(samana_msgs
  "/home/combinacijus/Documents/SamanaAutonomousRobot/ROS/samana_ws/src/samana_msgs/msg/ImuSmall.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/samana_msgs
)
_generate_msg_lisp(samana_msgs
  "/home/combinacijus/Documents/SamanaAutonomousRobot/ROS/samana_ws/src/samana_msgs/msg/ArmData.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/samana_msgs
)
_generate_msg_lisp(samana_msgs
  "/home/combinacijus/Documents/SamanaAutonomousRobot/ROS/samana_ws/src/samana_msgs/msg/Teleop.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/samana_msgs
)
_generate_msg_lisp(samana_msgs
  "/home/combinacijus/Documents/SamanaAutonomousRobot/ROS/samana_ws/src/samana_msgs/msg/Int16Array.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/samana_msgs
)
_generate_msg_lisp(samana_msgs
  "/home/combinacijus/Documents/SamanaAutonomousRobot/ROS/samana_ws/src/samana_msgs/msg/ImuCalib.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/samana_msgs
)

### Generating Services

### Generating Module File
_generate_module_lisp(samana_msgs
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/samana_msgs
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(samana_msgs_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(samana_msgs_generate_messages samana_msgs_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/combinacijus/Documents/SamanaAutonomousRobot/ROS/samana_ws/src/samana_msgs/msg/Vector4_32.msg" NAME_WE)
add_dependencies(samana_msgs_generate_messages_lisp _samana_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/combinacijus/Documents/SamanaAutonomousRobot/ROS/samana_ws/src/samana_msgs/msg/ArmCmd.msg" NAME_WE)
add_dependencies(samana_msgs_generate_messages_lisp _samana_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/combinacijus/Documents/SamanaAutonomousRobot/ROS/samana_ws/src/samana_msgs/msg/Vector3_32.msg" NAME_WE)
add_dependencies(samana_msgs_generate_messages_lisp _samana_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/combinacijus/Documents/SamanaAutonomousRobot/ROS/samana_ws/src/samana_msgs/msg/Bump.msg" NAME_WE)
add_dependencies(samana_msgs_generate_messages_lisp _samana_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/combinacijus/Documents/SamanaAutonomousRobot/ROS/samana_ws/src/samana_msgs/msg/ImuSmall.msg" NAME_WE)
add_dependencies(samana_msgs_generate_messages_lisp _samana_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/combinacijus/Documents/SamanaAutonomousRobot/ROS/samana_ws/src/samana_msgs/msg/ArmData.msg" NAME_WE)
add_dependencies(samana_msgs_generate_messages_lisp _samana_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/combinacijus/Documents/SamanaAutonomousRobot/ROS/samana_ws/src/samana_msgs/msg/Teleop.msg" NAME_WE)
add_dependencies(samana_msgs_generate_messages_lisp _samana_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/combinacijus/Documents/SamanaAutonomousRobot/ROS/samana_ws/src/samana_msgs/msg/Int16Array.msg" NAME_WE)
add_dependencies(samana_msgs_generate_messages_lisp _samana_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/combinacijus/Documents/SamanaAutonomousRobot/ROS/samana_ws/src/samana_msgs/msg/ImuCalib.msg" NAME_WE)
add_dependencies(samana_msgs_generate_messages_lisp _samana_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(samana_msgs_genlisp)
add_dependencies(samana_msgs_genlisp samana_msgs_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS samana_msgs_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(samana_msgs
  "/home/combinacijus/Documents/SamanaAutonomousRobot/ROS/samana_ws/src/samana_msgs/msg/Vector4_32.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/samana_msgs
)
_generate_msg_nodejs(samana_msgs
  "/home/combinacijus/Documents/SamanaAutonomousRobot/ROS/samana_ws/src/samana_msgs/msg/ArmCmd.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/samana_msgs
)
_generate_msg_nodejs(samana_msgs
  "/home/combinacijus/Documents/SamanaAutonomousRobot/ROS/samana_ws/src/samana_msgs/msg/Vector3_32.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/samana_msgs
)
_generate_msg_nodejs(samana_msgs
  "/home/combinacijus/Documents/SamanaAutonomousRobot/ROS/samana_ws/src/samana_msgs/msg/Bump.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/samana_msgs
)
_generate_msg_nodejs(samana_msgs
  "/home/combinacijus/Documents/SamanaAutonomousRobot/ROS/samana_ws/src/samana_msgs/msg/ImuSmall.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/samana_msgs
)
_generate_msg_nodejs(samana_msgs
  "/home/combinacijus/Documents/SamanaAutonomousRobot/ROS/samana_ws/src/samana_msgs/msg/ArmData.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/samana_msgs
)
_generate_msg_nodejs(samana_msgs
  "/home/combinacijus/Documents/SamanaAutonomousRobot/ROS/samana_ws/src/samana_msgs/msg/Teleop.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/samana_msgs
)
_generate_msg_nodejs(samana_msgs
  "/home/combinacijus/Documents/SamanaAutonomousRobot/ROS/samana_ws/src/samana_msgs/msg/Int16Array.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/samana_msgs
)
_generate_msg_nodejs(samana_msgs
  "/home/combinacijus/Documents/SamanaAutonomousRobot/ROS/samana_ws/src/samana_msgs/msg/ImuCalib.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/samana_msgs
)

### Generating Services

### Generating Module File
_generate_module_nodejs(samana_msgs
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/samana_msgs
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(samana_msgs_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(samana_msgs_generate_messages samana_msgs_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/combinacijus/Documents/SamanaAutonomousRobot/ROS/samana_ws/src/samana_msgs/msg/Vector4_32.msg" NAME_WE)
add_dependencies(samana_msgs_generate_messages_nodejs _samana_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/combinacijus/Documents/SamanaAutonomousRobot/ROS/samana_ws/src/samana_msgs/msg/ArmCmd.msg" NAME_WE)
add_dependencies(samana_msgs_generate_messages_nodejs _samana_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/combinacijus/Documents/SamanaAutonomousRobot/ROS/samana_ws/src/samana_msgs/msg/Vector3_32.msg" NAME_WE)
add_dependencies(samana_msgs_generate_messages_nodejs _samana_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/combinacijus/Documents/SamanaAutonomousRobot/ROS/samana_ws/src/samana_msgs/msg/Bump.msg" NAME_WE)
add_dependencies(samana_msgs_generate_messages_nodejs _samana_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/combinacijus/Documents/SamanaAutonomousRobot/ROS/samana_ws/src/samana_msgs/msg/ImuSmall.msg" NAME_WE)
add_dependencies(samana_msgs_generate_messages_nodejs _samana_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/combinacijus/Documents/SamanaAutonomousRobot/ROS/samana_ws/src/samana_msgs/msg/ArmData.msg" NAME_WE)
add_dependencies(samana_msgs_generate_messages_nodejs _samana_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/combinacijus/Documents/SamanaAutonomousRobot/ROS/samana_ws/src/samana_msgs/msg/Teleop.msg" NAME_WE)
add_dependencies(samana_msgs_generate_messages_nodejs _samana_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/combinacijus/Documents/SamanaAutonomousRobot/ROS/samana_ws/src/samana_msgs/msg/Int16Array.msg" NAME_WE)
add_dependencies(samana_msgs_generate_messages_nodejs _samana_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/combinacijus/Documents/SamanaAutonomousRobot/ROS/samana_ws/src/samana_msgs/msg/ImuCalib.msg" NAME_WE)
add_dependencies(samana_msgs_generate_messages_nodejs _samana_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(samana_msgs_gennodejs)
add_dependencies(samana_msgs_gennodejs samana_msgs_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS samana_msgs_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(samana_msgs
  "/home/combinacijus/Documents/SamanaAutonomousRobot/ROS/samana_ws/src/samana_msgs/msg/Vector4_32.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/samana_msgs
)
_generate_msg_py(samana_msgs
  "/home/combinacijus/Documents/SamanaAutonomousRobot/ROS/samana_ws/src/samana_msgs/msg/ArmCmd.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/samana_msgs
)
_generate_msg_py(samana_msgs
  "/home/combinacijus/Documents/SamanaAutonomousRobot/ROS/samana_ws/src/samana_msgs/msg/Vector3_32.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/samana_msgs
)
_generate_msg_py(samana_msgs
  "/home/combinacijus/Documents/SamanaAutonomousRobot/ROS/samana_ws/src/samana_msgs/msg/Bump.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/samana_msgs
)
_generate_msg_py(samana_msgs
  "/home/combinacijus/Documents/SamanaAutonomousRobot/ROS/samana_ws/src/samana_msgs/msg/ImuSmall.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/samana_msgs
)
_generate_msg_py(samana_msgs
  "/home/combinacijus/Documents/SamanaAutonomousRobot/ROS/samana_ws/src/samana_msgs/msg/ArmData.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/samana_msgs
)
_generate_msg_py(samana_msgs
  "/home/combinacijus/Documents/SamanaAutonomousRobot/ROS/samana_ws/src/samana_msgs/msg/Teleop.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/samana_msgs
)
_generate_msg_py(samana_msgs
  "/home/combinacijus/Documents/SamanaAutonomousRobot/ROS/samana_ws/src/samana_msgs/msg/Int16Array.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/samana_msgs
)
_generate_msg_py(samana_msgs
  "/home/combinacijus/Documents/SamanaAutonomousRobot/ROS/samana_ws/src/samana_msgs/msg/ImuCalib.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/samana_msgs
)

### Generating Services

### Generating Module File
_generate_module_py(samana_msgs
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/samana_msgs
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(samana_msgs_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(samana_msgs_generate_messages samana_msgs_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/combinacijus/Documents/SamanaAutonomousRobot/ROS/samana_ws/src/samana_msgs/msg/Vector4_32.msg" NAME_WE)
add_dependencies(samana_msgs_generate_messages_py _samana_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/combinacijus/Documents/SamanaAutonomousRobot/ROS/samana_ws/src/samana_msgs/msg/ArmCmd.msg" NAME_WE)
add_dependencies(samana_msgs_generate_messages_py _samana_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/combinacijus/Documents/SamanaAutonomousRobot/ROS/samana_ws/src/samana_msgs/msg/Vector3_32.msg" NAME_WE)
add_dependencies(samana_msgs_generate_messages_py _samana_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/combinacijus/Documents/SamanaAutonomousRobot/ROS/samana_ws/src/samana_msgs/msg/Bump.msg" NAME_WE)
add_dependencies(samana_msgs_generate_messages_py _samana_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/combinacijus/Documents/SamanaAutonomousRobot/ROS/samana_ws/src/samana_msgs/msg/ImuSmall.msg" NAME_WE)
add_dependencies(samana_msgs_generate_messages_py _samana_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/combinacijus/Documents/SamanaAutonomousRobot/ROS/samana_ws/src/samana_msgs/msg/ArmData.msg" NAME_WE)
add_dependencies(samana_msgs_generate_messages_py _samana_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/combinacijus/Documents/SamanaAutonomousRobot/ROS/samana_ws/src/samana_msgs/msg/Teleop.msg" NAME_WE)
add_dependencies(samana_msgs_generate_messages_py _samana_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/combinacijus/Documents/SamanaAutonomousRobot/ROS/samana_ws/src/samana_msgs/msg/Int16Array.msg" NAME_WE)
add_dependencies(samana_msgs_generate_messages_py _samana_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/combinacijus/Documents/SamanaAutonomousRobot/ROS/samana_ws/src/samana_msgs/msg/ImuCalib.msg" NAME_WE)
add_dependencies(samana_msgs_generate_messages_py _samana_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(samana_msgs_genpy)
add_dependencies(samana_msgs_genpy samana_msgs_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS samana_msgs_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/samana_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/samana_msgs
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(samana_msgs_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(samana_msgs_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/samana_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/samana_msgs
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(samana_msgs_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(samana_msgs_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/samana_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/samana_msgs
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(samana_msgs_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(samana_msgs_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/samana_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/samana_msgs
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(samana_msgs_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(samana_msgs_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/samana_msgs)
  install(CODE "execute_process(COMMAND \"/usr/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/samana_msgs\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/samana_msgs
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(samana_msgs_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(samana_msgs_generate_messages_py geometry_msgs_generate_messages_py)
endif()
