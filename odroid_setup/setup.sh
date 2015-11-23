#!/bin/bash

robot=$1
if [ -z $robot ]; then
	echo "robot arg must be set (ex: command.sh 01)"
	exit 1
fi

source ~/catkin_ws/params/set_this_ip.sh
source ~/catkin_ws/params/set_master_ip.sh

if [ $ROS_MASTER_IP = $THIS_IP ];then
	echo "- copy .bashrc"
	scp ~/catkin_ws/odroid_setup/.bashrc odroid@$ROBOTS_BASE_IP$robot:.bashrc
	
	echo "- deploy catkin_ws"
	bash ~/catkin_ws/deployment/catkin_ws.sh $robot
	
	echo "- add ssh key"
	ssh-copy-id odroid@$ROBOTS_BASE_IP$robot && ssh-keyscan -t rsa $ROBOTS_BASE_IP$robot > ~/.tmp_ssh_copy_id
	ssh_copy_id=$(cat ~/.tmp_ssh_copy_id)
	ssh_copy_id_grepped=`cat ~/.ssh/known_hosts | grep "$ssh_copy_id"`
	if [ -z "$ssh_copy_id_grepped" ]; then
		echo "ssh key not known"
		echo $ssh_copy_id >> ~/.ssh/known_hosts
	else
		echo "ssh key already known"
	fi
	
	echo "- setup on robot$robot"
	ssh odroid@$ROBOTS_BASE_IP$robot 'echo odroid|sudo -S bash /home/odroid/catkin_ws/odroid_setup/setup.sh '$robot''
else
	echo "  - change hostname to odroid$robot"
	echo "odroid"$robot > /etc/hostname
	
	echo "  - set md25 serial number"
	md25_serial=$(udevadm info -a --name /dev/ttyUSB* | grep "ATTRS{idVendor}==\"0403\"" -A24 | grep "ATTRS{serial}")
	md25_serial=`echo $md25_serial | cut -d '"' -f 2`
	echo "md25 serial found: $md25_serial"
	
	echo "  - write tty file"
	rm /etc/udev/rules.d/99-usb-serial.rules
	echo 'SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", ATTRS{serial}=="0001", SYMLINK+="rplidar"' > /etc/udev/rules.d/99-usb-serial.rules
	echo 'SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6001", ATTRS{serial}=="'$md25_serial'", SYMLINK+="md25"' >> /etc/udev/rules.d/99-usb-serial.rules
fi