#!/bin/bash

robot=$1
user=$USER

# to make a backup
#ssh odroid@192.168.150.1$robot 'rm -rf ~/catkin_ws_backup_before_deployment'
#ssh odroid@192.168.150.1$robot 'mv ~/catkin_ws ~/catkin_ws_backup_before_deployment'

# to sync clock
ssh odroid@192.168.150.1$robot 'echo odroid|sudo -S service ntp stop; echo odroid|sudo -S ntpdate 192.168.150.1'

rsync -r -avz --delete-after \
	--exclude '/catkin_ws/.git/' \
	--exclude '/catkin_ws/.settings/' \
	--exclude '/catkin_ws/src/amcl_test' \
	--exclude '/catkin_ws/src/blob_detect' \
	--exclude '/catkin_ws/src/gui_controller' \
	--exclude '/catkin_ws/src/gui_execution_diagram' \
	--exclude '/catkin_ws/src/gui_execution_viz' \
	--exclude '/catkin_ws/src/gui_scenario_builder' \
	--exclude '/catkin_ws/src/gui_scenario_db' \
	--exclude '/catkin_ws/src/gui_video_db' \
	--exclude '/catkin_ws/src/hector_navigation' \
	--exclude '/catkin_ws/src/launch_utils' \
	--exclude '/catkin_ws/src/local_path' \
	--exclude '/catkin_ws/src/obstacle_avoidance_test' \
	--exclude '/catkin_ws/src/path_checker' \
	--exclude '/catkin_ws/src/path_finding' \
	--exclude '/catkin_ws/src/robot_visualisation' \
	--exclude '/catkin_ws/src/rviz' \
	--exclude '/catkin_ws/src/simple_navigation_goals' \
	--exclude '/catkin_ws/src/stdr_server' \
    --exclude '/catkin_ws/src/stdr_gui' \
    --exclude '/catkin_ws/src/stdr_launchers' \
    --exclude '/catkin_ws/src/stdr_msgs' \
    --exclude '/catkin_ws/src/stdr_parser' \
    --exclude '/catkin_ws/src/stdr_resources' \
    --exclude '/catkin_ws/src/stdr_robot' \
    --exclude '/catkin_ws/src/stdr_samples' \
    --exclude '/catkin_ws/src/stdr_simulator' \
	--exclude '/catkin_ws/src/switch_launch_config' \
	--exclude '/catkin_ws/src/teleop_twist_keyboard' \
	--exclude '/catkin_ws/src/topic_router' \
	--exclude '/catkin_ws/bag/' \
	--exclude '/catkin_ws/devel/' \
	--exclude '/catkin_ws/build/' \
	--exclude '/catkin_ws/deployment/' \
	--exclude '/catkin_ws/build_isolated/' \
	--exclude '/catkin_ws/devel_isolated/' \
	--exclude '/catkin_ws/README.md' \
	--exclude '*.pyc' \
	--exclude '*.gitignore' \
	--exclude '*.project' \
	--exclude '*.pydevproject' \
	~/catkin_ws odroid@192.168.150.1$robot:~/

ssh odroid@192.168.150.1$robot 'cd ~/catkin_ws/;catkin_make'
#ssh odroid@192.168.150.1$robot 'cd ~/catkin_ws/src/hector_navigation/;rm -rf ./*/build;rosmake'

echo "deployment done on robot $robot"
