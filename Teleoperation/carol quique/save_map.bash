#!/bin/bash
source /opt/ros/humble/setup.bash
unset ROS_LOCALHOST_ONLY        # hay que hablar con el robot por la red
export ROS_DOMAIN_ID=7         # <-- número de dominio de TU robot
export TURTLEBOT3_MODEL=burger

n=1
while [ -e "$(printf 'map_%02d.yaml' $n)" ]; do n=$((n+1)); done
ros2 run nav2_map_server map_saver_cli -f "$(printf 'map_%02d' $n)" --ros-args -p save_map_timeout:=10.0
