#!/bin/bash

cd ~/catkin_ws/run/mapping && make clean; make

ROBOT=$1



user=$USER

#set time on odroid
ssh odroid@192.168.1.1$ROBOT 'echo odroid|sudo -S service ntp stop; echo odroid|sudo -S ntpdate 192.168.1.151'

cd ~/catkin_ws/bin/ && ./mapping $ROBOT
