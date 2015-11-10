#!/bin/bash

user=$USER

#set time on odroid
echo "Sync time robot 00"
ssh odroid@192.168.150.100 'echo odroid|sudo -S service ntp stop; echo odroid|sudo -S ntpdate 192.168.150.1'
#ssh odroid@192.168.150.101 'echo odroid|sudo -S service ntp stop; echo odroid|sudo -S ntpdate 192.168.150.1'
#ssh odroid@192.168.150.102 'echo odroid|sudo -S service ntp stop; echo odroid|sudo -S ntpdate 192.168.150.1'
echo "Sync time robot 03"
ssh odroid@192.168.150.103 'echo odroid|sudo -S service ntp stop; echo odroid|sudo -S ntpdate 192.168.150.1'
#ssh odroid@192.168.150.104 'echo odroid|sudo -S service ntp stop; echo odroid|sudo -S ntpdate 192.168.150.1'
#ssh odroid@192.168.150.105 'echo odroid|sudo -S service ntp stop; echo odroid|sudo -S ntpdate 192.168.150.1'
#ssh odroid@192.168.150.106 'echo odroid|sudo -S service ntp stop; echo odroid|sudo -S ntpdate 192.168.150.1'

#launch robot
roslaunch robot init_global.launch
