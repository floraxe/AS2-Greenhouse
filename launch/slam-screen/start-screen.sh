#!/usr/bin/env bash
set -e
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
/usr/bin/python3 -c 'import tkinter, PIL.ImageTk, numpy, rclpy, tf2_ros; from rtabmap_msgs.msg import MapGraph' || {
    echo 'Install display dependencies: sudo apt install python3-tk python3-pil.imagetk python3-numpy'
    exit 1
}
/usr/bin/python3 live_slam_screen.py --self-test
exec /usr/bin/python3 live_slam_screen.py
