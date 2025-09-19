cmake_minimum_required(VERSION 3.0.2)
project(rosbag_traj_replayer)

find_package(catkin REQUIRED COMPONENTS
  rospy
  rosbag
  sensor_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/replay_joint_states.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
