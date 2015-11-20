#!/bin/bash

robot=$1
if [ -z $robot ]; then
	echo "robot arg must be set (ex: command.sh 01)"
	exit 1
fi

ssh odroid@$ROBOTS_BASE_IP$robot 'ls -la /dev/rplidar;ls -la /dev/md25'
