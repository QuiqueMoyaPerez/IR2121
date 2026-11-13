#!/bin/bash
source /opt/ros/humble/setup.bash
export ROS_LOCALHOST_ONLY=1     # reproducción en local: no molestar al robot


ros2 bag play rosbag2_2026_11_09-18_57_06_0.db3 --clock
