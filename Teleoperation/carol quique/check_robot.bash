#!/bin/bash
source /opt/ros/humble/setup.bash
unset ROS_LOCALHOST_ONLY        # hay que hablar con el robot por la red
export ROS_DOMAIN_ID=7         # <-- número de dominio de TU robot
export TURTLEBOT3_MODEL=burger

# Comprobar que se ve el robot: deben salir /scan /odom /tf ...
ros2 topic list
