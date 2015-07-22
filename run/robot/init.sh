#!/bin/bash

robot=$1

user=$USER

#set time on odroid
ssh odroid@192.168.150.1$robot 'echo odroid|sudo -S service ntp stop; echo odroid|sudo -S ntpdate 192.168.150.1'

roslaunch robot init_bis.launch robot:=$robot
