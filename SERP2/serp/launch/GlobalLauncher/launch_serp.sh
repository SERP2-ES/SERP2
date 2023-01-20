#!/usr/bin/env bash

# Update application icon
mv serp.desktop serp.desktop-bak
sed -e "s,Icon=.*,Icon=$PWD/icon.png,g" serp.desktop-bak > serp.desktop
rm serp.desktop-bak
export SERP_PROJECT_PATH=~/catkin_ws/src/SERP/serp/
. ~/catkin_ws/devel/setup.bash

roscore &
sleep 3
#rosrun serp rpi_camera_node &
#rosrun serp arduino_bridge_node &
rosrun rosserial_python serial_node.py _port:=/dev/ttyUSB0 _baud:=57600 &
rosrun serp graphical_interface & 
rosrun serp vision_node.py & 
rosrun serp object_detection_node &
rosrun serp logic_node







