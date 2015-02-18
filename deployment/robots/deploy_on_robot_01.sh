#!/bin/bash

robot=01

ssh odroid@192.168.150.1$robot 'rm -rf ~/catkin_ws_backup_before_deployment'
ssh odroid@192.168.150.1$robot 'mv ~/catkin_ws ~/catkin_ws_backup_before_deployment'

rsync -r -v \
	--exclude '/catkin_ws/.git/' \
	--exclude '/catkin_ws/.settings/' \
	--exclude '/catkin_ws/src/blob_detect' \
	--exclude '/catkin_ws/src/gui_controller' \
	--exclude '/catkin_ws/src/gui_execution_diagram' \
	--exclude '/catkin_ws/src/gui_execution_viz' \
	--exclude '/catkin_ws/src/gui_scenario_builder' \
	--exclude '/catkin_ws/src/gui_scenario_db' \
	--exclude '/catkin_ws/src/launch_utils' \
	--exclude '/catkin_ws/src/rviz' \
	--exclude '/catkin_ws/src/teleop_twist_keyboard' \
	--exclude '/catkin_ws/bag/' \
	--exclude '/catkin_ws/devel/etc' \
	--exclude '/catkin_ws/devel/include' \
	--exclude '/catkin_ws/devel/devel' \
	--exclude '/catkin_ws/devel/lib' \
	--exclude '/catkin_ws/devel/share' \
	--exclude '/catkin_ws/build/' \
	--exclude '/catkin_ws/deployment/' \
	--exclude '/catkin_ws/README.md' \
	--exclude '*.pyc' \
	--exclude '*.gitignore' \
	--exclude '*.project' \
	--exclude '*.pydevproject' \
	~/catkin_ws odroid@192.168.150.1$robot:~/

#ssh odroid@192.168.150.1$robot 'bash -s' < ~/catkin_ws/deployment/robots/deploy_on_robot.sh

echo "deployment done, DO NOT FORGET TO CATKIN_MAKE"
