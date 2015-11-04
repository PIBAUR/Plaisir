#!/bin/bash

robot=$1

server_folder=$(cat ~/catkin_ws/params/myparams.yaml | grep video_db_path)
server_folder=`echo $server_folder| cut -d'"' -f 2`

robot_folder=$(cat ~/catkin_ws/params/myparams.yaml | grep robot_videos_path)
robot_folder=`echo $robot_folder| cut -d'"' -f 2`

rsync -r -avz \
	$server_folder/* odroid@192.168.150.1$robot:$robot_folder

echo "deployment done on robot $robot"

