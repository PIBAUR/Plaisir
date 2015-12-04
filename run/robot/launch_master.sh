#!/bin/bash

# generate .rviz
echo "generate .rviz for robots $robots"
python ~/catkin_ws/params/generate_init_global_rviz.py 00 01 02 03 04 05 06

#launch robot
roslaunch robot init_global.launch
