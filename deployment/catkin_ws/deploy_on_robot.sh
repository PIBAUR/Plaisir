#!/bin/bash

robot=$1

# to make a backup
#ssh odroid@192.168.150.1$robot 'rm -rf ~/catkin_ws_backup_before_deployment'
#ssh odroid@192.168.150.1$robot 'mv ~/catkin_ws ~/catkin_ws_backup_before_deployment'

rsync -r -avz --delete-after \
	--exclude '/catkin_ws/.git/' \
	--exclude '/catkin_ws/.settings/' \
	--exclude '/catkin_ws/src/blob_detect' \
	--exclude '/catkin_ws/src/path_finding' \
	--exclude '/catkin_ws/src/gui_controller' \
	--exclude '/catkin_ws/src/gui_execution_diagram' \
	--exclude '/catkin_ws/src/gui_execution_viz' \
	--exclude '/catkin_ws/src/gui_scenario_builder' \
	--exclude '/catkin_ws/src/gui_scenario_db' \
	--exclude '/catkin_ws/src/hector_navigation' \
	--exclude '/catkin_ws/src/launch_utils' \
	--exclude '/catkin_ws/src/rviz' \
	--exclude '/catkin_ws/src/teleop_twist_keyboard' \
	--exclude '/catkin_ws/bag/' \
	--exclude '/catkin_ws/devel/' \
	--exclude '/catkin_ws/build/' \
	--exclude '/catkin_ws/deployment/' \
	--exclude '/catkin_ws/README.md' \
	--exclude '*.pyc' \
	--exclude '*.gitignore' \
	--exclude '*.project' \
	--exclude '*.pydevproject' \
	~/catkin_ws odroid@192.168.150.1$robot:~/

ssh odroid@192.168.150.1$robot 'cd ~/catkin_ws/;catkin_make'
#ssh odroid@192.168.150.1$robot 'cd ~/catkin_ws/src/hector_navigation/;rm -rf ./*/build;rosmake'

echo "deployment done on robot $robot"
