
#include "obstacle_laserscan.h"

void lidarCallback(const sensor_msgs::LaserScan::ConstPtr& msg){
	front_stop.data=false;
	back_stop.data=false;

	//front obstacle
	for(size_t i = min_scan_range(front_angle,FRONT); i <=max_scan_range(front_angle,FRONT); i++)
	{
		if((msg->ranges[i] < front_dist_obstacle_max)
			&& (msg->ranges[i] > front_dist_obstacle_min)) //obstacle situated between 15cm and 30cm
		{
			if(front_stop.data == true)
				break;
			ROS_INFO_STREAM("angle #" << i << " : " << msg->ranges[i]);
			front_stop.data = true;
		}
	}
	//back obstacle
	for(size_t i = min_scan_range(back_angle,BACK); i <=360; i++)
	{
		if((msg->ranges[i] < back_dist_obstacle_max)
			&& (msg->ranges[i] > back_dist_obstacle_min)) //obstacle situated between 15cm and 30cm
		{
			if(back_stop.data == true)
				break;
			ROS_INFO_STREAM("angle #" << i << " : " << msg->ranges[i]);
			back_stop.data = true;
		}
	}
	// Other for due to discontinue range
	if(!back_stop.data)
	{
		for(size_t i = 0; i <= max_scan_range(back_angle,BACK); i++)
		{
			if((msg->ranges[i] < back_dist_obstacle_max)
				&& (msg->ranges[i] > back_dist_obstacle_min)) //obstacle situated between 15cm and 30cm
			{
				if(back_stop.data == true)
					break;
				ROS_INFO_STREAM("angle #" << i << " : " << msg->ranges[i]);
				back_stop.data = true;
			}
		}
	}
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "obstacle_laserscan_node");
	ros::NodeHandle n;

	/*** get rosparam ***/
	n.param<double>("/front_angle",front_angle,SCAN_RANGE);
	n.param<double>("/front_distance_obstacle_min",front_dist_obstacle_min,MIN_LENGTH);
	n.param<double>("/front_distance_obstacle_max",front_dist_obstacle_max,MAX_LENGTH);

	ROS_INFO_STREAM("Front obstacle detection angle range from " << min_scan_range(front_angle,FRONT)
					<< " to " << max_scan_range(front_angle,FRONT));

	n.param<double>("/back_angle",back_angle,SCAN_RANGE);
	n.param<double>("/back_distance_obstacle_min",back_dist_obstacle_min,MIN_LENGTH);
	n.param<double>("/back_distance_obstacle_max",back_dist_obstacle_max,MAX_LENGTH);
	ROS_INFO_STREAM("Front obstacle detection angle range from " << min_scan_range(back_angle,BACK)
						<< " to " << max_scan_range(back_angle,BACK));
	//Subscriber
	ros::Subscriber scan_sub = n.subscribe<sensor_msgs::LaserScan>("scan", 1, lidarCallback);
	//Publisher
	ros::Publisher front_stop_pub = n.advertise<std_msgs::Bool>("front_obstacle", 1);
	ros::Publisher back_stop_pub = n.advertise<std_msgs::Bool>("back_obstacle", 1);
	// frequency
	ros::Rate r(5);

	front_stop.data=false;
	back_stop.data=false;
	//ROS Loop
	while (ros::ok())
	{
		// message to be written
		front_stop_pub.publish(front_stop);
		back_stop_pub.publish(back_stop);
		ros::spinOnce();
		r.sleep();
	}

 	return 0;
}
