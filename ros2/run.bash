#!/bin/bash
source /opt/ros/humble/setup.bash
source /home/audrius/opw/cxx_bridge/ros2_server/install/setup.bash


xacro goblet.xacro > goblet.urdf
ros2 launch ./view.launch.py


