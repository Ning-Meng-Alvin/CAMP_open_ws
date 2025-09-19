cmake_minimum_required(VERSION 3.0.2)
project(moveit_franky_bridge)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  moveit_msgs
)

catkin_package()

# Install python nodes
catkin_install_python(PROGRAMS
  src/moveit_result_to_franky_goal.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
