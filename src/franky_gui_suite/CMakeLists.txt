cmake_minimum_required(VERSION 3.0.2)
project(franky_gui_suite)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
)


catkin_python_setup()

catkin_package()


catkin_install_python(PROGRAMS
  scripts/robot_ui_suite.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(PROGRAMS
  scripts/robot_ui_suite.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
