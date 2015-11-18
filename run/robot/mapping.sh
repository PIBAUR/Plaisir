#!/bin/bash

cd ~/catkin_ws/run/mapping && make clean; make

ROBOT=$1



user=$USER

#set time on odroid
<<<<<<< HEAD
ssh odroid@192.168.1.1$ROBOT 'echo odroid|sudo -S service ntp stop; echo odroid|sudo -S ntpdate 192.168.1.151'
=======
ssh odroid@192.168.1.1$ROBOT 'echo odroid|sudo -S service ntp stop; echo odroid|sudo -S ntpdate 192.168.1.1'
>>>>>>> branch 'master' of https://github.com/digitalarti/notre_bon_plaisir.git

cd ~/catkin_ws/bin/ && ./mapping $ROBOT
