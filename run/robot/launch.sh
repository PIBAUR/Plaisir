#!/bin/bash

#bash ~/catkin_ws/run/robot/script_before_launch_for_robot.sh 00
#bash ~/catkin_ws/run/robot/script_before_launch_for_robot.sh 01
#bash ~/catkin_ws/run/robot/script_before_launch_for_robot.sh 02
bash ~/catkin_ws/run/robot/script_before_launch_for_robot.sh 03
#bash ~/catkin_ws/run/robot/script_before_launch_for_robot.sh 04
#bash ~/catkin_ws/run/robot/script_before_launch_for_robot.sh 05
#bash ~/catkin_ws/run/robot/script_before_launch_for_robot.sh 06

#launch robot
roslaunch robot init_global.launch
