cmake_minimum_required(VERSION 3.0.2)
project(rosbag_recoder)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  std_srvs
)

catkin_package()


catkin_install_python(PROGRAMS
  src/recorder_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
  PATTERN ".git" EXCLUDE
)
