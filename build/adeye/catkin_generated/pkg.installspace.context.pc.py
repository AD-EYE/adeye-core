# generated from catkin/cmake/template/pkg.context.pc.in
CATKIN_PACKAGE_PREFIX = ""
PROJECT_PKG_CONFIG_INCLUDE_DIRS = "${prefix}/include".split(';') if "${prefix}/include" != "" else []
PROJECT_CATKIN_DEPENDS = "roscpp;grid_map_core;grid_map_msgs;nav_msgs;std_msgs;geometry_msgs;autoware_msgs;op_planner;op_ros_helpers;tf2_ros;adeye_utils".replace(';', ' ')
PKG_CONFIG_LIBRARIES_WITH_PREFIX = "-ladeye".split(';') if "-ladeye" != "" else []
PROJECT_NAME = "adeye"
PROJECT_SPACE_DIR = "/home/adeye/adeye-core/install/adeye"
PROJECT_VERSION = "0.0.0"
