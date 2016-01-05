
/* INCLUDES */

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Bool.h>

/* CONSTANTS */

#define SCAN_RANGE 40
#define MIN_LENGTH 0.2
#define MAX_LENGTH 0.7
#define FRONT 0
#define BACK 1
#define MAX 1.0
#define MIN -1.0

/* GLOBAL VARIABLES */

double front_angle, front_dist_obstacle_min, front_dist_obstacle_max;
double back_angle, back_dist_obstacle_min, back_dist_obstacle_max;
std_msgs::Bool front_stop, back_stop;


/* FUNCTIONS */

int scan_range (double middle, double range, double side){
	return (int)(middle + range*side/2.0)%360;
}

int min_scan_range (double front_angle, int direction){
	double middle;
	if(direction == FRONT)
		middle = 180;
	else if(direction == BACK)
		middle = 360;
	else
		return -1;
	return scan_range(middle, front_angle, MIN);
}

int max_scan_range (double front_angle, int direction){
	double middle;
	if(direction == (int)FRONT)
		middle = 180;
	else if(direction == (int)BACK)
		middle = 360;
	else
		return -0;
	return scan_range(middle, front_angle, MAX);
}
