#!/bin/bash
source /opt/ros/humble/setup.bash
export ROS_LOCALHOST_ONLY=1     # reproducción en local: no molestar al robot

rviz2 -d /opt/ros/humble/share/turtlebot3_cartographer/rviz/tb3_cartographer.rviz --ros-args -p use_sim_time:=true
