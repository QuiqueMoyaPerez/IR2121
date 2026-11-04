#!/bin/bash
source /opt/ros/humble/setup.bash
unset ROS_LOCALHOST_ONLY        # hay que hablar con el robot por la red
export ROS_DOMAIN_ID=7         # <-- número de dominio de TU robot
export TURTLEBOT3_MODEL=burger

ros2 bag record -o slam_robot \
  /map /map_updates /odom /robot_description /scan \
  /scan_matched_points2 /submap_list /tf /tf_static
