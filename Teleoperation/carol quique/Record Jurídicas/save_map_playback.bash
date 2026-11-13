#!/bin/bash
# Ejecutar DENTRO de la carpeta del método: guarda map_01, map_02... cada 90 s
source /opt/ros/humble/setup.bash
export ROS_LOCALHOST_ONLY=1
while true; do
  n=1
  while [ -e "$(printf 'map_%02d.yaml' $n)" ]; do n=$((n+1)); done
  ros2 run nav2_map_server map_saver_cli -f "$(printf 'map_%02d' $n)" \
     --ros-args -p save_map_timeout:=10.0
  sleep 90
done
