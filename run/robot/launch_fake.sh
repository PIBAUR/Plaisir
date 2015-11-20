#!/bin/bash

robot=$1
if [ -z $robot ]; then
	echo "robot arg must be set (ex: command.sh 01)"
	exit 1
fi

#launch robot
roslaunch robot init_fake.launch robot:=$robot
