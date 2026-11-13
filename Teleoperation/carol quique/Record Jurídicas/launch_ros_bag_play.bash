#!/bin/bash
# Uso: ./launch_ros_bag_play.bash [velocidad]     (por defecto 1.0)
source /opt/ros/humble/setup.bash
export ROS_LOCALHOST_ONLY=1
DIR="$(cd "$(dirname "$0")" && pwd)"
ros2 bag play "$DIR/rosbag2_2026_11_09-18_57_06_0.db3" --clock --rate "${1:-1.0}"
