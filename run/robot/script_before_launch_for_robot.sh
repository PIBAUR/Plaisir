#!/bin/bash

robot=$1
if [ -z $robot ]; then
	echo "robot arg must be set (ex: command.sh 01)"
	exit 1
fi

echo "Sync time robot $robot"
ssh odroid@$ROBOTS_BASE_IP$robot 'echo odroid|sudo -S service ntp stop; echo odroid|sudo -S ntpdate $ROS_MASTER_IP'

echo "prevent screen to shutdown $robot and move mouse to right bottom corner"
ssh odroid@$ROBOTS_BASE_IP$robot 'export DISPLAY=:0;xset s off -dpms;xdotool mousemove 1920 1080'

