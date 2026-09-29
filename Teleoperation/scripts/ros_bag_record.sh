#!/bin/bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=10
export TURTLEBOT3_MODEL=burger
ros2 bag record /clock /odom /tf /tf_static /scan /map

