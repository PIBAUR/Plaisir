#!/bin/bash

user=$USER

ROS_MASTER_IP=${ROS_MASTER_URI:7:13}

#set time on odroid
echo "Sync time robot 01"
ssh odroid@$ROBOTS_BASE_IP"01" 'echo odroid|sudo -S service ntp stop; echo odroid|sudo -S ntpdate $ROS_MASTER_IP'

#launch robot
roslaunch robot init_global.launch
