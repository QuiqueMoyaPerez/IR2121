#!/bin/bash
# Solo para la ejecución final del método elegido. Ejecutar dentro de su carpeta.
source /opt/ros/humble/setup.bash
export ROS_LOCALHOST_ONLY=1
ros2 bag record -o slam_robot_completo \
  /map /odom /robot_description /scan /tf /tf_static
