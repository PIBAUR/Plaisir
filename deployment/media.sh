#!/bin/bash

robot=$1
if [ -z $robot ]; then
	echo "robot arg must be set (ex: command.sh 01)"
	exit 1
fi

server_folder=$(cat ~/catkin_ws/params/myparams.yaml | grep video_db_path)
server_folder=`echo $server_folder| cut -d'"' -f 2`
eval server_folder=$server_folder

robot_folder=$(cat ~/catkin_ws/params/myparams.yaml | grep robot_videos_path)
robot_folder=`echo $robot_folder| cut -d'"' -f 2`
eval robot_folder=$robot_folder

rsync -r -avz $server_folder/* odroid@$ROBOTS_BASE_IP$robot:$robot_folder

echo "deployment done on robot $robot"

