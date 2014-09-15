#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
//#include <math>

#define PI 3.14159265359
#define DU 0.10
#define F  30
#define LENGTH 5

geometry_msgs::PoseArray path;

void compute_path()
{
    geometry_msgs::Pose p;
    
    p.position.x = 0;
    p.position.y = 0;
    p.position.z = 0;
    p.orientation.x = 0;
    p.orientation.y = 0;
    p.orientation.z = 0;
    p.orientation.w = 1;
    
    path.poses.push_back(p);
    double x = 0, y = 0, th = 0, dist = 0;
    double t = 0, dy = 0 , dx = 0;
    while(dist < LENGTH)
    {
        //compute next pose
        x += DU;
        y = (-1 + cos(2*PI*t/F)) * 0.5;
        
        dx = x - p.position.x;
        dy = y - p.position.y;
        
        th = atan2(dy , dx);
        
        //compute length of path
        dist += sqrt(dx*dx + dy*dy);
        t++;
        
        //set pose
        p.position.x = x;
        p.position.y = y;        
        p.orientation.z = sin(th/2);
        p.orientation.w = cos(th/2);
        
        //add pose to path
        path.poses.push_back(p);
    }
}    
 
int main(int argc, char **argv)
{
    ros::init(argc, argv, "path_publisher");
	ros::NodeHandle n;
    
	ros::Publisher path_pub = n.advertise<geometry_msgs::PoseArray>("path",1);
    
    path.header.frame_id = "map";
    
    ros::Rate loop(1);
    ros::spinOnce();
    loop.sleep();
    
    compute_path();
    path_pub.publish(path);
    
	while(ros::ok()){
		ros::spinOnce();
		loop.sleep();
	}
	ros::spin();
    
    return 0;
    
}
