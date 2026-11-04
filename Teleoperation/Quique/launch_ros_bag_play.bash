#!/bin/bash
source /opt/ros/humble/setup.bash
export ROS_LOCALHOST_ONLY=1     # reproducción en local: no molestar al robot

ros2 bag play slam_robot --clock
