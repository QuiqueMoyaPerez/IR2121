#!/bin/bash
source /opt/ros/humble/setup.bash
export ROS_LOCALHOST_ONLY=1
export TURTLEBOT3_MODEL=burger
DIR="$(cd "$(dirname "$0")" && pwd)"

# Copia de la configuración de SLAM Toolbox leyendo /scan_fixed y con el alcance del LIDAR
sed -e 's#scan_topic:.*#scan_topic: /scan_fixed#' \
    -e 's#max_laser_range:.*#max_laser_range: 3.5#' \
    /opt/ros/humble/share/slam_toolbox/config/mapper_params_online_async.yaml \
    > /tmp/mapper_params_fixed.yaml

python3 "$DIR/scan_fixer.py" --ros-args -p use_sim_time:=true &
FIX=$!
trap "kill $FIX 2>/dev/null" EXIT

ros2 launch slam_toolbox online_async_launch.py use_sim_time:=True \
    slam_params_file:=/tmp/mapper_params_fixed.yaml
