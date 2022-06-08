# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "musi_care: 1 messages, 2 services")

set(MSG_I_FLAGS "-Imusi_care:/home/jwaad/catkin_ws/src/musi_care/msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(musi_care_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/jwaad/catkin_ws/src/musi_care/msg/SongData.msg" NAME_WE)
add_custom_target(_musi_care_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "musi_care" "/home/jwaad/catkin_ws/src/musi_care/msg/SongData.msg" ""
)

get_filename_component(_filename "/home/jwaad/catkin_ws/src/musi_care/srv/qt_command.srv" NAME_WE)
add_custom_target(_musi_care_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "musi_care" "/home/jwaad/catkin_ws/src/musi_care/srv/qt_command.srv" ""
)

get_filename_component(_filename "/home/jwaad/catkin_ws/src/musi_care/srv/sound_player_srv.srv" NAME_WE)
add_custom_target(_musi_care_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "musi_care" "/home/jwaad/catkin_ws/src/musi_care/srv/sound_player_srv.srv" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(musi_care
  "/home/jwaad/catkin_ws/src/musi_care/msg/SongData.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/musi_care
)

### Generating Services
_generate_srv_cpp(musi_care
  "/home/jwaad/catkin_ws/src/musi_care/srv/qt_command.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/musi_care
)
_generate_srv_cpp(musi_care
  "/home/jwaad/catkin_ws/src/musi_care/srv/sound_player_srv.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/musi_care
)

### Generating Module File
_generate_module_cpp(musi_care
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/musi_care
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(musi_care_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(musi_care_generate_messages musi_care_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/jwaad/catkin_ws/src/musi_care/msg/SongData.msg" NAME_WE)
add_dependencies(musi_care_generate_messages_cpp _musi_care_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jwaad/catkin_ws/src/musi_care/srv/qt_command.srv" NAME_WE)
add_dependencies(musi_care_generate_messages_cpp _musi_care_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jwaad/catkin_ws/src/musi_care/srv/sound_player_srv.srv" NAME_WE)
add_dependencies(musi_care_generate_messages_cpp _musi_care_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(musi_care_gencpp)
add_dependencies(musi_care_gencpp musi_care_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS musi_care_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(musi_care
  "/home/jwaad/catkin_ws/src/musi_care/msg/SongData.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/musi_care
)

### Generating Services
_generate_srv_eus(musi_care
  "/home/jwaad/catkin_ws/src/musi_care/srv/qt_command.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/musi_care
)
_generate_srv_eus(musi_care
  "/home/jwaad/catkin_ws/src/musi_care/srv/sound_player_srv.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/musi_care
)

### Generating Module File
_generate_module_eus(musi_care
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/musi_care
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(musi_care_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(musi_care_generate_messages musi_care_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/jwaad/catkin_ws/src/musi_care/msg/SongData.msg" NAME_WE)
add_dependencies(musi_care_generate_messages_eus _musi_care_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jwaad/catkin_ws/src/musi_care/srv/qt_command.srv" NAME_WE)
add_dependencies(musi_care_generate_messages_eus _musi_care_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jwaad/catkin_ws/src/musi_care/srv/sound_player_srv.srv" NAME_WE)
add_dependencies(musi_care_generate_messages_eus _musi_care_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(musi_care_geneus)
add_dependencies(musi_care_geneus musi_care_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS musi_care_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(musi_care
  "/home/jwaad/catkin_ws/src/musi_care/msg/SongData.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/musi_care
)

### Generating Services
_generate_srv_lisp(musi_care
  "/home/jwaad/catkin_ws/src/musi_care/srv/qt_command.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/musi_care
)
_generate_srv_lisp(musi_care
  "/home/jwaad/catkin_ws/src/musi_care/srv/sound_player_srv.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/musi_care
)

### Generating Module File
_generate_module_lisp(musi_care
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/musi_care
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(musi_care_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(musi_care_generate_messages musi_care_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/jwaad/catkin_ws/src/musi_care/msg/SongData.msg" NAME_WE)
add_dependencies(musi_care_generate_messages_lisp _musi_care_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jwaad/catkin_ws/src/musi_care/srv/qt_command.srv" NAME_WE)
add_dependencies(musi_care_generate_messages_lisp _musi_care_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jwaad/catkin_ws/src/musi_care/srv/sound_player_srv.srv" NAME_WE)
add_dependencies(musi_care_generate_messages_lisp _musi_care_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(musi_care_genlisp)
add_dependencies(musi_care_genlisp musi_care_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS musi_care_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(musi_care
  "/home/jwaad/catkin_ws/src/musi_care/msg/SongData.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/musi_care
)

### Generating Services
_generate_srv_nodejs(musi_care
  "/home/jwaad/catkin_ws/src/musi_care/srv/qt_command.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/musi_care
)
_generate_srv_nodejs(musi_care
  "/home/jwaad/catkin_ws/src/musi_care/srv/sound_player_srv.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/musi_care
)

### Generating Module File
_generate_module_nodejs(musi_care
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/musi_care
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(musi_care_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(musi_care_generate_messages musi_care_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/jwaad/catkin_ws/src/musi_care/msg/SongData.msg" NAME_WE)
add_dependencies(musi_care_generate_messages_nodejs _musi_care_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jwaad/catkin_ws/src/musi_care/srv/qt_command.srv" NAME_WE)
add_dependencies(musi_care_generate_messages_nodejs _musi_care_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jwaad/catkin_ws/src/musi_care/srv/sound_player_srv.srv" NAME_WE)
add_dependencies(musi_care_generate_messages_nodejs _musi_care_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(musi_care_gennodejs)
add_dependencies(musi_care_gennodejs musi_care_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS musi_care_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(musi_care
  "/home/jwaad/catkin_ws/src/musi_care/msg/SongData.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/musi_care
)

### Generating Services
_generate_srv_py(musi_care
  "/home/jwaad/catkin_ws/src/musi_care/srv/qt_command.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/musi_care
)
_generate_srv_py(musi_care
  "/home/jwaad/catkin_ws/src/musi_care/srv/sound_player_srv.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/musi_care
)

### Generating Module File
_generate_module_py(musi_care
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/musi_care
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(musi_care_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(musi_care_generate_messages musi_care_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/jwaad/catkin_ws/src/musi_care/msg/SongData.msg" NAME_WE)
add_dependencies(musi_care_generate_messages_py _musi_care_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jwaad/catkin_ws/src/musi_care/srv/qt_command.srv" NAME_WE)
add_dependencies(musi_care_generate_messages_py _musi_care_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jwaad/catkin_ws/src/musi_care/srv/sound_player_srv.srv" NAME_WE)
add_dependencies(musi_care_generate_messages_py _musi_care_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(musi_care_genpy)
add_dependencies(musi_care_genpy musi_care_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS musi_care_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/musi_care)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/musi_care
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(musi_care_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/musi_care)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/musi_care
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(musi_care_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/musi_care)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/musi_care
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(musi_care_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/musi_care)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/musi_care
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(musi_care_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/musi_care)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/musi_care\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/musi_care
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(musi_care_generate_messages_py std_msgs_generate_messages_py)
endif()
