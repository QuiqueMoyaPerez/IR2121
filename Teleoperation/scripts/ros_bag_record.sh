#!/bin/bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=2
export TURTLEBOT3_MODEL=burger
ros2 bag record /clock /odom /tf /tf_static

