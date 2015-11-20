#!/bin/bash

source ~/catkin_ws/params/set_this_ip.sh
source ~/catkin_ws/params/set_robots_base_ip.sh

export ROS_MASTER_IP=$THIS_IP
export ROS_IP=$THIS_IP
export ROS_HOSTNAME=$ROS_MASTER_IP
export ROS_MASTER_URI=http://$ROS_MASTER_IP:11311