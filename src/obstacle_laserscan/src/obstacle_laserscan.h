
/* INCLUDES */

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Bool.h>

/* CONSTANTS */

#define SCAN_RANGE 40
#define MIN_LENGTH 0.2
#define MAX_LENGTH 0.7

/* GLOBAL VARIABLES */

double front_angle,dist_obstacle_min,dist_obstacle_max;
std_msgs::Bool stop;

/* FUNCTIONS */

double min_scan_range (double front_angle){

return (180 - front_angle /2);

}
double max_scan_range (double front_angle){

return (180 + front_angle /2);

}
