#!/bin/bash

robot=$1
if [ -z $robot ]; then
	echo "robot arg must be set (ex: command.sh 01)"
	exit 1
fi

# to make a backup
#ssh odroid@$ROBOTS_BASE_IP$robot 'rm -rf ~/catkin_ws_backup_before_deployment'
#ssh odroid@$ROBOTS_BASE_IP$robot 'mv ~/catkin_ws ~/catkin_ws_backup_before_deployment'

# to sync clock
if [ "-p" = $2 ]; then
	echo "no time sync because of arg -p"
else
	ssh odroid@$ROBOTS_BASE_IP$robot 'echo odroid|sudo -S service ntp stop; echo odroid|sudo -S ntpdate $ROS_MASTER_IP'
fi

# to set ros_master_uri IP
source ~/catkin_ws/params/set_this_ip.sh
rm ~/catkin_ws/params/set_master_ip.sh -f
echo "export ROS_MASTER_IP=$THIS_IP" >> ~/catkin_ws/params/set_master_ip.sh

# rsync
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
	--exclude '/catkin_ws/src/occupancy_grid_utils' \
	--exclude '/catkin_ws/src/python_bindings_tutorial' \
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
	--exclude '/catkin_ws/maps/' \
	--exclude '/catkin_ws/run/' \
	--exclude '/catkin_ws/build/' \
	--exclude '/catkin_ws/deployment/' \
	--exclude '/catkin_ws/build_isolated/' \
	--exclude '/catkin_ws/devel_isolated/' \
	--exclude '/catkin_ws/README.md' \
	--exclude '*.pyc' \
	--exclude '*.gitignore' \
	--exclude '*.project' \
	--exclude '*.pydevproject' \
	~/catkin_ws odroid@$ROBOTS_BASE_IP$robot:~/

if [ "-p" = $2 ]; then
	echo "no time sync because of arg -p"
else
	ssh odroid@$ROBOTS_BASE_IP$robot 'cd ~/catkin_ws/;catkin_make'
#ssh odroid@$ROBOTS_BASE_IP$robot 'cd ~/catkin_ws/src/hector_navigation/;rm -rf ./*/build;rosmake'
fi

echo "deployment done on robot $robot"
