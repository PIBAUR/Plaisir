#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseArray.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
//#include <math>

#define PI 3.14159265359


geometry_msgs::PoseArray path;
float x_robot;
float y_robot;
double theta_robot;
int index_path;
int size_path;
double du;

void PathCB(const geometry_msgs::PoseArray &msg)
{
    path = msg;
    index_path = 0;
    size_path = path.poses.size();
} 

void compute_cmd(tf::TransformListener &tf_listener, double &lin, double &ang)
{
    //ros::NodeHandle n2;
	//tf::TransformListener tf_listener;
    tf::StampedTransform tf_robot;

    try
    {
      tf_listener.lookupTransform("base_link", path.header.frame_id, ros::Time(0), tf_robot);
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
      return;
    }
    //tf::Vector3 vect =  (tf_robot.getOrigin());
    x_robot = tf_robot.getOrigin().x();
    y_robot =  tf_robot.getOrigin().y();
    double val_q_robot =  2 * tf_robot.getRotation().z() * tf_robot.getRotation().w();
    theta_robot = tf::getYaw(tf_robot.getRotation());
    
    double dx;
    double dy;
    
    double dth;
    
    double x_des, y_des, theta_des;
    x_des =  path.poses[index_path].position.x;
    y_des =  path.poses[index_path].position.y;
    double val_q_des =  (2 * path.poses[index_path].orientation.z * path.poses[index_path].orientation.w);
    theta_des =  asin(val_q_des);
    
    dx = x_des - x_robot;
    dy = y_des - y_robot;
    du = sqrt(dx*dx + dy*dy);
    dth = theta_des - theta_robot;
    




    //a
    double alpha = atan2(dy,dx);
    alpha +=theta_robot;
    /**while(alpha > PI)
    {
        alpha-=2*PI;
    }
    
    while(alpha < -PI)
    {
        alpha+=2*PI;
    }


        while(alpha > PI)
        {
            alpha-=2*PI;
        }

        while(alpha < -PI)
        {
            alpha+=2*PI;
        }*/
    double cmd_lin, cmd_ang;
    ROS_WARN("plop : %f\t %f\t %f ",alpha,theta_robot,atan2(dy,dx));
    ang = alpha;
    lin = 0.01;
}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "path_follower");
	ros::NodeHandle n;
    
	ros::Subscriber path_sub = n.subscribe("path", 1, PathCB);
	ros::Publisher cmd_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);
	tf::TransformListener tf_listener;

    geometry_msgs::Twist cmd;
    
    cmd.linear.y=0;
    cmd.linear.z=0;
    cmd.angular.x=0;
    cmd.angular.y=0;
    
    index_path = 0;
    size_path = 0;
    du = 10;
    
    ros::Rate loop(15);
	while(ros::ok()){
        if(size_path !=0 && index_path<size_path)
        {
            if(du<0.15)
            {
                index_path++;
                du=10;
            }
            else
            {
                compute_cmd(tf_listener, cmd.linear.x, cmd.angular.z);
                cmd_pub.publish(cmd);
            }
        }
		ros::spinOnce();
		loop.sleep();
	}
	ros::spin();
    return 0;
    
}
