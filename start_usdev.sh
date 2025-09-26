#!/bin/bash

# 设置OpenCV环境变量，避免GStreamer问题
export OPENCV_VIDEOIO_PRIORITY_V4L2=1
export OPENCV_VIDEOIO_DEBUG=1

# 设置视频设备权限
sudo chmod 666 /dev/video* 2>/dev/null || true

# 启动ROS系统
cd /home/camp/CampUsers/Pei/open_ws
source devel/setup.bash
roslaunch usdev_framegrab stream_usimg_dyncfg.launch




