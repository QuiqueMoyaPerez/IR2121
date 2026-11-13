#!/bin/bash
source /opt/ros/humble/setup.bash
unset ROS_LOCALHOST_ONLY        # hay que hablar con el robot por la red
export ROS_DOMAIN_ID=7         # <-- número de dominio de TU robot
export TURTLEBOT3_MODEL=burger

ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=False
