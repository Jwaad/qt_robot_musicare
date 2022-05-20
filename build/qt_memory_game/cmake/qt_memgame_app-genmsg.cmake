# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "qt_memgame_app: 0 messages, 1 services")

set(MSG_I_FLAGS "-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(qt_memgame_app_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/qtrobot/catkin_ws/src/qt_memory_game/srv/suspend.srv" NAME_WE)
add_custom_target(_qt_memgame_app_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "qt_memgame_app" "/home/qtrobot/catkin_ws/src/qt_memory_game/srv/suspend.srv" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages

### Generating Services
_generate_srv_cpp(qt_memgame_app
  "/home/qtrobot/catkin_ws/src/qt_memory_game/srv/suspend.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/qt_memgame_app
)

### Generating Module File
_generate_module_cpp(qt_memgame_app
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/qt_memgame_app
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(qt_memgame_app_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(qt_memgame_app_generate_messages qt_memgame_app_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/qtrobot/catkin_ws/src/qt_memory_game/srv/suspend.srv" NAME_WE)
add_dependencies(qt_memgame_app_generate_messages_cpp _qt_memgame_app_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(qt_memgame_app_gencpp)
add_dependencies(qt_memgame_app_gencpp qt_memgame_app_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS qt_memgame_app_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages

### Generating Services
_generate_srv_eus(qt_memgame_app
  "/home/qtrobot/catkin_ws/src/qt_memory_game/srv/suspend.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/qt_memgame_app
)

### Generating Module File
_generate_module_eus(qt_memgame_app
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/qt_memgame_app
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(qt_memgame_app_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(qt_memgame_app_generate_messages qt_memgame_app_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/qtrobot/catkin_ws/src/qt_memory_game/srv/suspend.srv" NAME_WE)
add_dependencies(qt_memgame_app_generate_messages_eus _qt_memgame_app_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(qt_memgame_app_geneus)
add_dependencies(qt_memgame_app_geneus qt_memgame_app_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS qt_memgame_app_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages

### Generating Services
_generate_srv_lisp(qt_memgame_app
  "/home/qtrobot/catkin_ws/src/qt_memory_game/srv/suspend.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/qt_memgame_app
)

### Generating Module File
_generate_module_lisp(qt_memgame_app
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/qt_memgame_app
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(qt_memgame_app_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(qt_memgame_app_generate_messages qt_memgame_app_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/qtrobot/catkin_ws/src/qt_memory_game/srv/suspend.srv" NAME_WE)
add_dependencies(qt_memgame_app_generate_messages_lisp _qt_memgame_app_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(qt_memgame_app_genlisp)
add_dependencies(qt_memgame_app_genlisp qt_memgame_app_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS qt_memgame_app_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages

### Generating Services
_generate_srv_nodejs(qt_memgame_app
  "/home/qtrobot/catkin_ws/src/qt_memory_game/srv/suspend.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/qt_memgame_app
)

### Generating Module File
_generate_module_nodejs(qt_memgame_app
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/qt_memgame_app
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(qt_memgame_app_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(qt_memgame_app_generate_messages qt_memgame_app_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/qtrobot/catkin_ws/src/qt_memory_game/srv/suspend.srv" NAME_WE)
add_dependencies(qt_memgame_app_generate_messages_nodejs _qt_memgame_app_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(qt_memgame_app_gennodejs)
add_dependencies(qt_memgame_app_gennodejs qt_memgame_app_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS qt_memgame_app_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages

### Generating Services
_generate_srv_py(qt_memgame_app
  "/home/qtrobot/catkin_ws/src/qt_memory_game/srv/suspend.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/qt_memgame_app
)

### Generating Module File
_generate_module_py(qt_memgame_app
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/qt_memgame_app
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(qt_memgame_app_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(qt_memgame_app_generate_messages qt_memgame_app_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/qtrobot/catkin_ws/src/qt_memory_game/srv/suspend.srv" NAME_WE)
add_dependencies(qt_memgame_app_generate_messages_py _qt_memgame_app_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(qt_memgame_app_genpy)
add_dependencies(qt_memgame_app_genpy qt_memgame_app_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS qt_memgame_app_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/qt_memgame_app)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/qt_memgame_app
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(qt_memgame_app_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/qt_memgame_app)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/qt_memgame_app
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(qt_memgame_app_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/qt_memgame_app)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/qt_memgame_app
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(qt_memgame_app_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/qt_memgame_app)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/qt_memgame_app
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(qt_memgame_app_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/qt_memgame_app)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/qt_memgame_app\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/qt_memgame_app
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(qt_memgame_app_generate_messages_py std_msgs_generate_messages_py)
endif()
