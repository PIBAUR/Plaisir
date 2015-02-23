#include "path_follower.h"



void PathFollower::pathCB(const geometry_msgs::PoseArray &msg)
{
    path_ = msg;
    index_path_ = 0;
    size_path_ = path_.poses.size();
} 


void PathFollower::computeCmd(double &lin, double &ang)
{
    tf::StampedTransform tf_robot;

    try
    {
    	//TODO: replace "map" by path.header.frame_id
        tf_listener_.lookupTransform("/map", "base_link", ros::Time(0), tf_robot);
    }
    catch (tf::TransformException ex)
    {
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
        return;
    }

    double x_robot, y_robot, theta_robot;
    double dx, dy;
    double x_des, y_des;
    double alpha;
    
    x_robot = tf_robot.getOrigin().x();
    y_robot =  tf_robot.getOrigin().y();
    theta_robot = tf::getYaw(tf_robot.getRotation());

    x_des =  path_.poses[index_path_].position.x;
    y_des =  path_.poses[index_path_].position.y;
    //theta_des = tf::getYaw(path.poses[index_path].orientation);
    
    dx = x_des - x_robot;
    dy = y_des - y_robot;
    du_=sqrt(dx*dx+dy*dy);

    alpha = atan2(dy,dx);

    ROS_INFO_STREAM("Target #"<<index_path_<<" : ["<<x_des<<"|"<<y_des<<std::endl<<"Robot : ["<<x_robot<<"|"<<y_robot<<"]"<<" du : "<<du_);

    ang = alpha-theta_robot;
    while(ang<-PI)
            ang+=2*PI;
    while(ang>=PI)
            ang-=2*PI;
    ang*=K_TH;
    lin = 0.15;
}



void PathFollower::spinOnce()
{
    geometry_msgs::Twist cmd;
    cmd.linear.y=0;
    cmd.linear.z=0;
    cmd.angular.x=0;
    cmd.angular.y=0;


    if(size_path_ !=0 && index_path_<size_path_)
    {
        if(du_<0.10)
        {
            index_path_++;
            du_=10;
        }
        computeCmd(cmd.linear.x, cmd.angular.z);
        cmd_pub_.publish(cmd);
    }
    else if( index_path_=size_path_)
    {
        cmd.linear.x = 0;
        cmd.angular.z = 0;
        cmd_pub_.publish(cmd);
    }
    cpt_++;
    if(cpt_>6)
    {
        std_msgs::Float64 ratio;
        ratio.data = 1.0*index_path_/size_path_;
        ratio_pub_.publish(ratio);
        cpt_=0;
    }
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "path_follower");
    ros::NodeHandle nh;
    PathFollower pf(nh);

    ros::Subscriber sub = nh.subscribe("path", 1, &PathFollower::pathCB, &pf);


    /*
    ros::Subscriber path_sub = n.subscribe("path", 1, PathCB);
    ros::Publisher cmd_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    ros::Publisher path_fb = n.advertise<std_msgs::Float64>("path_feedback", 1);
    tf::TransformListener tf_listener;
    */

    ros::Rate loop(LOOP_RATE);
    while(ros::ok()){
        pf.spinOnce();
        ros::spinOnce();
        loop.sleep();
    }
    ros::spin();
    return 0;
    
}
