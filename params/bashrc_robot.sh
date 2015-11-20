#!/bin/bash

source ~/catkin_ws/params/set_this_ip.sh
source ~/catkin_ws/params/set_master_ip.sh

export ROS_MASTER_URI=http://$ROS_MASTER_IP:11311
export ROS_HOSTNAME=$THIS_IP
export ROS_IP=$THIS_IP
