# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "adeye_msgs: 2 messages, 0 services")

set(MSG_I_FLAGS "-Iadeye_msgs:/home/adeye/adeye-core/AD-EYE/ROS_Packages/src/adeye_msgs/msg;-Istd_msgs:/opt/ros/kinetic/share/std_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/kinetic/share/geometry_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(adeye_msgs_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/adeye/adeye-core/AD-EYE/ROS_Packages/src/adeye_msgs/msg/categorized_poses.msg" NAME_WE)
add_custom_target(_adeye_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "adeye_msgs" "/home/adeye/adeye-core/AD-EYE/ROS_Packages/src/adeye_msgs/msg/categorized_poses.msg" "geometry_msgs/Quaternion:geometry_msgs/Pose:adeye_msgs/categorized_pose:std_msgs/Header:geometry_msgs/Point"
)

get_filename_component(_filename "/home/adeye/adeye-core/AD-EYE/ROS_Packages/src/adeye_msgs/msg/categorized_pose.msg" NAME_WE)
add_custom_target(_adeye_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "adeye_msgs" "/home/adeye/adeye-core/AD-EYE/ROS_Packages/src/adeye_msgs/msg/categorized_pose.msg" "geometry_msgs/Quaternion:geometry_msgs/Pose:std_msgs/Header:geometry_msgs/Point"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(adeye_msgs
  "/home/adeye/adeye-core/AD-EYE/ROS_Packages/src/adeye_msgs/msg/categorized_poses.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg;/home/adeye/adeye-core/AD-EYE/ROS_Packages/src/adeye_msgs/msg/categorized_pose.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/adeye_msgs
)
_generate_msg_cpp(adeye_msgs
  "/home/adeye/adeye-core/AD-EYE/ROS_Packages/src/adeye_msgs/msg/categorized_pose.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/adeye_msgs
)

### Generating Services

### Generating Module File
_generate_module_cpp(adeye_msgs
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/adeye_msgs
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(adeye_msgs_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(adeye_msgs_generate_messages adeye_msgs_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/adeye/adeye-core/AD-EYE/ROS_Packages/src/adeye_msgs/msg/categorized_poses.msg" NAME_WE)
add_dependencies(adeye_msgs_generate_messages_cpp _adeye_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/adeye/adeye-core/AD-EYE/ROS_Packages/src/adeye_msgs/msg/categorized_pose.msg" NAME_WE)
add_dependencies(adeye_msgs_generate_messages_cpp _adeye_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(adeye_msgs_gencpp)
add_dependencies(adeye_msgs_gencpp adeye_msgs_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS adeye_msgs_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(adeye_msgs
  "/home/adeye/adeye-core/AD-EYE/ROS_Packages/src/adeye_msgs/msg/categorized_poses.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg;/home/adeye/adeye-core/AD-EYE/ROS_Packages/src/adeye_msgs/msg/categorized_pose.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/adeye_msgs
)
_generate_msg_eus(adeye_msgs
  "/home/adeye/adeye-core/AD-EYE/ROS_Packages/src/adeye_msgs/msg/categorized_pose.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/adeye_msgs
)

### Generating Services

### Generating Module File
_generate_module_eus(adeye_msgs
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/adeye_msgs
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(adeye_msgs_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(adeye_msgs_generate_messages adeye_msgs_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/adeye/adeye-core/AD-EYE/ROS_Packages/src/adeye_msgs/msg/categorized_poses.msg" NAME_WE)
add_dependencies(adeye_msgs_generate_messages_eus _adeye_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/adeye/adeye-core/AD-EYE/ROS_Packages/src/adeye_msgs/msg/categorized_pose.msg" NAME_WE)
add_dependencies(adeye_msgs_generate_messages_eus _adeye_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(adeye_msgs_geneus)
add_dependencies(adeye_msgs_geneus adeye_msgs_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS adeye_msgs_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(adeye_msgs
  "/home/adeye/adeye-core/AD-EYE/ROS_Packages/src/adeye_msgs/msg/categorized_poses.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg;/home/adeye/adeye-core/AD-EYE/ROS_Packages/src/adeye_msgs/msg/categorized_pose.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/adeye_msgs
)
_generate_msg_lisp(adeye_msgs
  "/home/adeye/adeye-core/AD-EYE/ROS_Packages/src/adeye_msgs/msg/categorized_pose.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/adeye_msgs
)

### Generating Services

### Generating Module File
_generate_module_lisp(adeye_msgs
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/adeye_msgs
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(adeye_msgs_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(adeye_msgs_generate_messages adeye_msgs_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/adeye/adeye-core/AD-EYE/ROS_Packages/src/adeye_msgs/msg/categorized_poses.msg" NAME_WE)
add_dependencies(adeye_msgs_generate_messages_lisp _adeye_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/adeye/adeye-core/AD-EYE/ROS_Packages/src/adeye_msgs/msg/categorized_pose.msg" NAME_WE)
add_dependencies(adeye_msgs_generate_messages_lisp _adeye_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(adeye_msgs_genlisp)
add_dependencies(adeye_msgs_genlisp adeye_msgs_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS adeye_msgs_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(adeye_msgs
  "/home/adeye/adeye-core/AD-EYE/ROS_Packages/src/adeye_msgs/msg/categorized_poses.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg;/home/adeye/adeye-core/AD-EYE/ROS_Packages/src/adeye_msgs/msg/categorized_pose.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/adeye_msgs
)
_generate_msg_nodejs(adeye_msgs
  "/home/adeye/adeye-core/AD-EYE/ROS_Packages/src/adeye_msgs/msg/categorized_pose.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/adeye_msgs
)

### Generating Services

### Generating Module File
_generate_module_nodejs(adeye_msgs
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/adeye_msgs
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(adeye_msgs_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(adeye_msgs_generate_messages adeye_msgs_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/adeye/adeye-core/AD-EYE/ROS_Packages/src/adeye_msgs/msg/categorized_poses.msg" NAME_WE)
add_dependencies(adeye_msgs_generate_messages_nodejs _adeye_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/adeye/adeye-core/AD-EYE/ROS_Packages/src/adeye_msgs/msg/categorized_pose.msg" NAME_WE)
add_dependencies(adeye_msgs_generate_messages_nodejs _adeye_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(adeye_msgs_gennodejs)
add_dependencies(adeye_msgs_gennodejs adeye_msgs_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS adeye_msgs_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(adeye_msgs
  "/home/adeye/adeye-core/AD-EYE/ROS_Packages/src/adeye_msgs/msg/categorized_poses.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg;/home/adeye/adeye-core/AD-EYE/ROS_Packages/src/adeye_msgs/msg/categorized_pose.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/adeye_msgs
)
_generate_msg_py(adeye_msgs
  "/home/adeye/adeye-core/AD-EYE/ROS_Packages/src/adeye_msgs/msg/categorized_pose.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/adeye_msgs
)

### Generating Services

### Generating Module File
_generate_module_py(adeye_msgs
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/adeye_msgs
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(adeye_msgs_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(adeye_msgs_generate_messages adeye_msgs_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/adeye/adeye-core/AD-EYE/ROS_Packages/src/adeye_msgs/msg/categorized_poses.msg" NAME_WE)
add_dependencies(adeye_msgs_generate_messages_py _adeye_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/adeye/adeye-core/AD-EYE/ROS_Packages/src/adeye_msgs/msg/categorized_pose.msg" NAME_WE)
add_dependencies(adeye_msgs_generate_messages_py _adeye_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(adeye_msgs_genpy)
add_dependencies(adeye_msgs_genpy adeye_msgs_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS adeye_msgs_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/adeye_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/adeye_msgs
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(adeye_msgs_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(adeye_msgs_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/adeye_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/adeye_msgs
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(adeye_msgs_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(adeye_msgs_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/adeye_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/adeye_msgs
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(adeye_msgs_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(adeye_msgs_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/adeye_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/adeye_msgs
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(adeye_msgs_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(adeye_msgs_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/adeye_msgs)
  install(CODE "execute_process(COMMAND \"/usr/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/adeye_msgs\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/adeye_msgs
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(adeye_msgs_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(adeye_msgs_generate_messages_py geometry_msgs_generate_messages_py)
endif()
