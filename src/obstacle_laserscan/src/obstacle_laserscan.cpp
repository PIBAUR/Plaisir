#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Bool.h>

//*****************Global Variables********************
sensor_msgs::LaserScan scan_data; 
size_t node_count=225;
size_t node_count2=180;
//*****************headers**********************
void lidarCallback(const sensor_msgs::LaserScan::ConstPtr& laserScan){

scan_data.header.frame_id=scan_data.header.frame_id;

        // values' copy
	for(size_t i =135 ; i < node_count2; i++) // the angle goes from 135 degree to 180 degree (45 degrees to the right)
		{       
			scan_data.ranges[i] = laserScan->ranges[i];
	 	 }
	for(size_t i =180 ; i < node_count; i++) // the angle goes from 180 degree to 225 degrees (45 degrees to the left)
		{       
			scan_data.ranges[i] = laserScan->ranges[i];
	 	 }	
 
}


//*************************************** 
int main(int argc, char** argv){

ros::init(argc, argv, "obstacle_laserscan_node");
    
ros::NodeHandle n;

//Subscriber
ros::Subscriber lidar_sub = n.subscribe<sensor_msgs::LaserScan>("scan", 50, lidarCallback);
//Publisher
ros::Publisher scan_pub = n.advertise<std_msgs::Bool>("scan_obstacle", 1);
// frequency
ros::Rate r(5);
//ROS Loop
 	while (ros::ok())
		{
			std_msgs::Bool stop;
			stop.data=false;
                        //***************************************************
                        scan_data.ranges.resize(node_count); // Important to resize to the max value (360)
                        //*****************************************************

		for(size_t i =135 ; i < node_count2; i++) 
		{       
			if((scan_data.ranges[i]<0.3)&&(scan_data.ranges[i]>0.15)) //obstacle situated between 15cm and 30cm
				{       
                      		 				stop.data=true;
                        	}
	 	 }
		for(size_t i =180 ; i < node_count; i++) 
		{       
			if((scan_data.ranges[i]<0.3)&&(scan_data.ranges[i]>0.15))//obstacle situated between 15cm and 30cm
				{       

                         		 				stop.data=true;
                        	}
	 	 }
          
// message to be written 
scan_pub.publish(stop);
ros::spinOnce();
r.sleep();
//ros::spinOnce(); 
		}
return 0;
   }
