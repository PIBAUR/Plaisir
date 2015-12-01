#!/bin/bash

export LAUNCH_ROBOT00=false
export launch_robot00=false
export LAUNCH_ROBOT01=false
export LAUNCH_ROBOT02=false
export LAUNCH_ROBOT03=false
export LAUNCH_ROBOT04=false
export LAUNCH_ROBOT05=false
export LAUNCH_ROBOT06=false

robots=""

for var in "$@"
do
    if [ "$var" = 00 ]; then
    	export LAUNCH_ROBOT00=true
    	if [ "-p" = $1 ]; then
			echo "no script before launch because of arg -p"
		else
			bash ~/catkin_ws/run/robot/script_before_launch_for_robot.sh 00
		fi
		robots="$robot robot00"
    fi
    
    if [ "$var" = 01 ]; then
    	export LAUNCH_ROBOT01=true
    	if [ "-p" = $1 ]; then
			echo "no script before launch because of arg -p"
		else
			bash ~/catkin_ws/run/robot/script_before_launch_for_robot.sh 01
		fi
		robots="$robot robot01"
    fi
    
    if [ "$var" = 02 ]; then
    	export LAUNCH_ROBOT02=true
    	if [ "-p" = $1 ]; then
			echo "no script before launch because of arg -p"
		else
			bash ~/catkin_ws/run/robot/script_before_launch_for_robot.sh 02
		fi
		robots="$robot robot02"
    fi
    
    if [ "$var" = 03 ]; then
    	export LAUNCH_ROBOT03=true
    	if [ "-p" = $1 ]; then
			echo "no script before launch because of arg -p"
		else
			bash ~/catkin_ws/run/robot/script_before_launch_for_robot.sh 03
		fi
		robots="$robot robot03"
    fi
    
    if [ "$var" = 04 ]; then
    	export LAUNCH_ROBOT04=true
    	if [ "-p" = $1 ]; then
			echo "no script before launch because of arg -p"
		else
			bash ~/catkin_ws/run/robot/script_before_launch_for_robot.sh 04
		fi
		robots="$robot robot04"
    fi
    
    if [ "$var" = 05 ]; then
    	export LAUNCH_ROBOT05=true
    	if [ "-p" = $1 ]; then
			echo "no script before launch because of arg -p"
		else
			bash ~/catkin_ws/run/robot/script_before_launch_for_robot.sh 05
		fi
		robots="$robot robot05"
    fi
    
    if [ "$var" = 06 ]; then
    	export LAUNCH_ROBOT06=true
    	if [ "-p" = $1 ]; then
			echo "no script before launch because of arg -p"
		else
			bash ~/catkin_ws/run/robot/script_before_launch_for_robot.sh 06
		fi
		robots="$robot robot06"
    fi
done

# generate .rviz
python ~/catkin_ws/params/generate_init_global_rviz.py $robots

#launch robot
roslaunch robot init_global.launch
