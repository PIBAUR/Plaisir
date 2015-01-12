#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseArray.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
//#include <math>

#define PI 3.14159265359
#define K_TH 5.0


geometry_msgs::PoseArray path;

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
    tf::StampedTransform tf_robot;

    try
    {
        tf_listener.lookupTransform(path.header.frame_id,"base_link", ros::Time(0), tf_robot);
    }
    catch (tf::TransformException ex)
    {
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
        return;
    }

    double x_robot, y_robot, theta_robot;
    double dx, dy, dth;
    double x_des, y_des, theta_des;
    double alpha;
    
    x_robot = tf_robot.getOrigin().x();
    y_robot =  tf_robot.getOrigin().y();
    theta_robot = tf::getYaw(tf_robot.getRotation());

    x_des =  path.poses[index_path].position.x;
    y_des =  path.poses[index_path].position.y;
    theta_des = tf::getYaw(path.poses[index_path].orientation);
    
    dx = x_des - x_robot;
    dy = y_des - y_robot;
    du=sqrt(dx*dx+dy*dy);
    dth = theta_des - theta_robot;

    alpha = atan2(dy,dx);

    ROS_INFO_STREAM("robot|alpha|r+a : "<<theta_robot<<"  |  "<<alpha<<"  |  "<<alpha-theta_robot);
    ROS_INFO_STREAM("du|dth : "<<du<<"  |  "<<dth);
    ROS_WARN_STREAM("Target #"<<index_path<<" : ["<<x_des<<"|"<<y_des<<std::endl<<"Robot : ["<<x_robot<<"|"<<y_robot<<"]");
    double cmd_lin, cmd_ang;
    //ROS_WARN("plop : %f\t %f\t %f ",alpha,theta_robot,atan2(dy,dx));
    ang = alpha-theta_robot;
    while(ang<-PI)
            ang+=2*PI;
    while(ang>=PI)
            ang-=2*PI;
    ang*=K_TH;

    lin = 0.15;
    /*if(abs(ang)<PI/2)
        lin = 0.05;
    else
        lin = 0.0;*/
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
    
    ros::Rate loop(60);
    while(ros::ok()){
        if(size_path !=0 && index_path<size_path)
        {
            if(du<0.10)
            {
                index_path++;
                du=10;
            }
            compute_cmd(tf_listener, cmd.linear.x, cmd.angular.z);
            cmd_pub.publish(cmd);
        }
        else if( index_path=size_path)
                {
                    cmd.linear.x = 0;
                    cmd.angular.z = 0;
                    cmd_pub.publish(cmd);
                }

        ros::spinOnce();
        loop.sleep();
    }
    ros::spin();
    return 0;
    
}
