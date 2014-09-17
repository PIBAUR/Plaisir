# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "test_suivi: 0 messages, 1 services")

set(MSG_I_FLAGS "-Istd_msgs:/opt/ros/groovy/share/std_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/groovy/share/geometry_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(genlisp REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(test_suivi_generate_messages ALL)

#
#  langs = gencpp;genlisp;genpy
#

### Section generating for lang: gencpp
### Generating Messages

### Generating Services
_generate_srv_cpp(test_suivi
  "/home/artlab/catkin_ws/src/test_suivi/srv/SendPose.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/groovy/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/groovy/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/groovy/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/groovy/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/groovy/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/test_suivi
)

### Generating Module File
_generate_module_cpp(test_suivi
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/test_suivi
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(test_suivi_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(test_suivi_generate_messages test_suivi_generate_messages_cpp)

# target for backward compatibility
add_custom_target(test_suivi_gencpp)
add_dependencies(test_suivi_gencpp test_suivi_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS test_suivi_generate_messages_cpp)

### Section generating for lang: genlisp
### Generating Messages

### Generating Services
_generate_srv_lisp(test_suivi
  "/home/artlab/catkin_ws/src/test_suivi/srv/SendPose.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/groovy/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/groovy/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/groovy/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/groovy/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/groovy/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/test_suivi
)

### Generating Module File
_generate_module_lisp(test_suivi
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/test_suivi
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(test_suivi_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(test_suivi_generate_messages test_suivi_generate_messages_lisp)

# target for backward compatibility
add_custom_target(test_suivi_genlisp)
add_dependencies(test_suivi_genlisp test_suivi_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS test_suivi_generate_messages_lisp)

### Section generating for lang: genpy
### Generating Messages

### Generating Services
_generate_srv_py(test_suivi
  "/home/artlab/catkin_ws/src/test_suivi/srv/SendPose.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/groovy/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/groovy/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/groovy/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/groovy/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/groovy/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/test_suivi
)

### Generating Module File
_generate_module_py(test_suivi
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/test_suivi
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(test_suivi_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(test_suivi_generate_messages test_suivi_generate_messages_py)

# target for backward compatibility
add_custom_target(test_suivi_genpy)
add_dependencies(test_suivi_genpy test_suivi_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS test_suivi_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/test_suivi)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/test_suivi
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
add_dependencies(test_suivi_generate_messages_cpp std_msgs_generate_messages_cpp)
add_dependencies(test_suivi_generate_messages_cpp geometry_msgs_generate_messages_cpp)

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/test_suivi)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/test_suivi
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
add_dependencies(test_suivi_generate_messages_lisp std_msgs_generate_messages_lisp)
add_dependencies(test_suivi_generate_messages_lisp geometry_msgs_generate_messages_lisp)

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/test_suivi)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/test_suivi\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/test_suivi
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
add_dependencies(test_suivi_generate_messages_py std_msgs_generate_messages_py)
add_dependencies(test_suivi_generate_messages_py geometry_msgs_generate_messages_py)
