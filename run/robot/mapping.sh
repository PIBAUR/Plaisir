#!/bin/bash

robot=$1
if [ -z $robot ]; then
	echo "robot arg must be set (ex: command.sh 01)"
	exit 1
fi

cd ~/catkin_ws/run/mapping && make clean; make

#set time on odroid
ssh odroid@$ROBOTS_BASE_IP$robot 'echo odroid|sudo -S service ntp stop; echo odroid|sudo -S ntpdate $ROS_MASTER_IP'

cd ~/catkin_ws/bin/ && ./mapping $robot
