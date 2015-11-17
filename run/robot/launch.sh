#!/bin/bash

user=$USER

THIS_IP=192.168.1.150:11311

#set time on odroid
#echo "Sync time robot 00"
#ssh odroid@192.168.1.100 'echo odroid|sudo -S service ntp stop; echo odroid|sudo -S ntpdate 192.168.1.1'
#ssh odroid@192.168.1.100 'echo odroid|export ROS_MASTER_URI=$THIS_IP'
echo "Sync time robot 03"
ssh odroid@192.168.1.103 'echo odroid|sudo -S service ntp stop; echo odroid|sudo -S ntpdate 192.168.1.1'
ssh odroid@192.168.1.103 'echo odroid|export ROS_MASTER_URI=$THIS_IP'

#launch robot
roslaunch robot init_global.launch
