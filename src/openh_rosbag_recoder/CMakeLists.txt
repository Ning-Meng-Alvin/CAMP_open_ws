cmake_minimum_required(VERSION 3.0.2)
project(openh_rosbag_recoder)

# Find catkin packages needed to build this package.
# The `dynamic_reconfigure` dependency is essential for generating
# the parameter configuration files from your .cfg file.
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  dynamic_reconfigure
)

#---------------------------------------------------------------------
# Dynamic Reconfigure Setup
#---------------------------------------------------------------------
# This command finds your .cfg file and generates the necessary
# Python module (e.g., RecorderParamsConfig) that your node imports.
generate_dynamic_reconfigure_options(
  cfg/RecorderParams.cfg
)
#---------------------------------------------------------------------

#---------------------------------------------------------------------
# Catkin Package Declaration
#---------------------------------------------------------------------
# This section declares the package's runtime dependencies.
# These are the packages that are needed when your node is executed.
catkin_package(
  # This package does not provide any C++ libraries.
  # LIBRARIES

  # This package does not provide any C++ include directories.
  # INCLUDE_DIRS

  # Runtime dependencies
  CATKIN_DEPENDS rospy std_msgs dynamic_reconfigure
)
#---------------------------------------------------------------------

#---------------------------------------------------------------------
# Install Python Executable Scripts (Recommended)
#---------------------------------------------------------------------
# This command installs the python script into the standard ROS binary location,
# making it accessible via `rosrun` after an install (`catkin_make install`).
# This is a best practice for distributing packages.
catkin_install_python(PROGRAMS
  scripts/openh_recorder_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
#---------------------------------------------------------------------