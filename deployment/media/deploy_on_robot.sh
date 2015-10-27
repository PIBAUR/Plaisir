#!/bin/bash

robot=$1

folder=$(cat ~/catkin_ws/params/myparams.yaml | grep video_db_path)
${'prout':1:1}

#rsync -r -avz --delete-after \
#	$folder odroid@192.168.150.1$robot:~/.notrebonplaisir/videos

#echo "deployment done on robot $robot"

