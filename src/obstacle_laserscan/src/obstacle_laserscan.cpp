<<<<<<< HEAD
#include "obstacle_laserscan.h"
=======
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Bool.h>

#define SCAN_RANGE 40
#define MIN_LENGTH 0.2
#define MAX_LENGTH 0.7
#define MIN_SCAN_RANGE (180 - SCAN_RANGE / 2)
#define MAX_SCAN_RANGE (180 + SCAN_RANGE / 2)

std_msgs::Bool stop;
>>>>>>> 8f6434d776eea3be7463ede6d8a4ee5a64067ca0

void lidarCallback(const sensor_msgs::LaserScan::ConstPtr& msg){
	stop.data=false;
        
	for(size_t i = min_scan_range(front_angle); i <=max_scan_range(front_angle); i++)
	{
		if((msg->ranges[i] < dist_obstacle_max) && (msg->ranges[i] > dist_obstacle_min)) //obstacle situated between 15cm and 30cm
		{
			ROS_INFO_STREAM("angle #" << i << " : " << msg->ranges[i]);
			stop.data = true;
			break;
		}
	}
 
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "obstacle_laserscan_node");
	ros::NodeHandle n;

        /*** get rosparam ***/
    	n.param<double>("/front_angle",front_angle,SCAN_RANGE);
    	n.param<double>("/dist_obstacle_min",dist_obstacle_min,MIN_LENGTH);
    	n.param<double>("/dist_obstacle_max",dist_obstacle_max,MAX_LENGTH);
	//Subscriber
	ros::Subscriber lidar_sub = n.subscribe<sensor_msgs::LaserScan>("scan", 1, lidarCallback);
	//Publisher
	ros::Publisher scan_pub = n.advertise<std_msgs::Bool>("front_obstacle", 1);
	// frequency
	ros::Rate r(5);

	stop.data=false;

	//ROS Loop
	while (ros::ok())
	{
		// message to be written
		scan_pub.publish(stop);
		ros::spinOnce();
		r.sleep();
	}

 	return 0;
}
