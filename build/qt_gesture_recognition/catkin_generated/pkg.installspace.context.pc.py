# generated from catkin/cmake/template/pkg.context.pc.in
CATKIN_PACKAGE_PREFIX = ""
PROJECT_PKG_CONFIG_INCLUDE_DIRS = "${prefix}/include".split(';') if "${prefix}/include" != "" else []
PROJECT_CATKIN_DEPENDS = "qt_gesture_controller;qt_motors_controller;qt_nuitrack_app".replace(';', ' ')
PKG_CONFIG_LIBRARIES_WITH_PREFIX = "-lqt_gesturegame_app;-lstd_msgs;-lmessage_runtime".split(';') if "-lqt_gesturegame_app;-lstd_msgs;-lmessage_runtime" != "" else []
PROJECT_NAME = "qt_gesturegame_app"
PROJECT_SPACE_DIR = "/home/qtrobot/catkin_ws/install"
PROJECT_VERSION = "1.0.1"
