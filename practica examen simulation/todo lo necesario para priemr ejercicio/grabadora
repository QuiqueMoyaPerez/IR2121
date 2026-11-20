#!/bin/bash
source /opt/ros/humble/setup.bash
export ROS_LOCALHOST_ONLY=1
export TURTLEBOT3_MODEL=waffle_pi
ros2 bag record -o rosbag_teleop \
  /clock \
  /tf \
  /tf_static \
  /odom \
  /scan \
  /robot_description \
  /joint_states
