#!/bin/bash

cd ~/catkin_ws/run/mapping && make clean; make

ROBOT=$1

user=$USER

#set time on odroid
ssh odroid@$ROBOTS_BASE_IP$ROBOT 'echo odroid|sudo -S service ntp stop; echo odroid|sudo -S ntpdate $ROS_MASTER_IP'

cd ~/catkin_ws/bin/ && ./mapping $ROBOT
