#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>

#include <iostream>
#include <sstream>
#include <ctime>
#include <fstream>

#define SPEED_MAX   1.0
#define SPEED_STEP  0.05
#define WAIT_TIME   5.0

/*****************CLASS DEFINE******************/

class LinearSpeedTester
{
public:
    ros::NodeHandle nh_;
    ros::Publisher cmd_pub_;
    tf::TransformListener tf_listener_;
    tf::StampedTransform tf_robot_old_;
    tf::StampedTransform tf_robot_new_;
    std::ofstream ofilestream_;
    double speed_max_;
    double speed_step_;

public:
    LinearSpeedTester(ros::NodeHandle nh);
    ~LinearSpeedTester();
    void testSpeed(float const &lin);
    void performLinearTest();
    void sleep(float const &sec);
    void saveTest(float const &lin);
};


LinearSpeedTester::LinearSpeedTester(ros::NodeHandle nh):
    nh_(nh)
{
    ros::NodeHandle nh_private("~");
    cmd_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    nh_private.param<double>("speed_max",speed_max_,SPEED_MAX);
    nh_private.param<double>("speed_step",speed_step_,SPEED_STEP);
    ROS_INFO_STREAM("Linear test from -"<<speed_max_<<"to"<<speed_max_<<" with step of "<<speed_step_);

    // current date/time based on current system
    std::time_t now = time(0);
    std::tm *ltm = localtime(&now);
    //create file name
    std::stringstream file_name;
    file_name<<std::getenv("HOME")<<"/catkin_ws/data/test_linear_speed_";
    file_name<<"20"<<ltm->tm_year-100<<"-"; //year
    file_name<<1 + ltm->tm_mon<<"-";        //month
    file_name<<ltm->tm_mday<<"_";           //day
    file_name<<1+ltm->tm_hour<<"-";         //hours
    if(1+ltm->tm_min < 10)
        file_name<<"0"<<1+ltm->tm_min<<"-"; //minutes
    else
        file_name<<1+ltm->tm_min<<"-";      //minutes
    if(1+ltm->tm_sec<10)
        file_name<<"0"<<1+ltm->tm_sec;      //seconds
    else
        file_name<<1+ltm->tm_sec;      //seconds
    //open file
    try
    {
        ofilestream_.open((char*)file_name.str().c_str(), std::ofstream::out | std::ofstream::app);
    }
    catch (tf::TransformException ex)
    {
        ROS_ERROR("%s",ex.what());
        exit(EXIT_FAILURE);
    }
}


LinearSpeedTester::~LinearSpeedTester()
{
    ofilestream_.close();
    geometry_msgs::Twist cmd;
    cmd.linear.x = 0.0;
    cmd.linear.y = 0.0;
    cmd.linear.z = 0.0;
    cmd.angular.x = 0.0;
    cmd.angular.y = 0.0;
    cmd.angular.z = 0.0;
    //send message
    cmd_pub_.publish(cmd);
}



void LinearSpeedTester::testSpeed(float const &lin)
{
    //creat twist message
    geometry_msgs::Twist cmd;
    cmd.linear.x = lin;
    cmd.linear.y = 0.0;
    cmd.linear.z = 0.0;
    cmd.angular.x = 0.0;
    cmd.angular.y = 0.0;
    cmd.angular.z = 0.0;

    //send message
    cmd_pub_.publish(cmd);
    ros::Duration(0.10).sleep();

    //get robot pose
    ros::spinOnce();
    try
    {
        tf_listener_.lookupTransform("odom", "base_link", ros::Time(0), tf_robot_old_);
    }
    catch (tf::TransformException ex)
    {
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
        return;
    }

    //wait
    sleep(WAIT_TIME);

    //get new robot pose
    //get robot pose
    ros::spinOnce();
    try
    {
        tf_listener_.lookupTransform("odom", "base_link", ros::Time(0), tf_robot_new_);
    }
    catch (tf::TransformException ex)
    {
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
        return;
    }

    //stop robot
    cmd_pub_.publish(cmd); //Just to make rqt plot easier to read
    cmd.linear.x = 0.0;
    cmd_pub_.publish(cmd);

    //save data
    saveTest(lin);
}

void LinearSpeedTester::sleep(float const &sec)
{
    ros::Time goal_time = ros::Time::now() + ros::Duration(sec);
    while(ros::Time::now() < goal_time && ros::ok())
    {
        ros::spinOnce();
        ros::Duration(0.2).sleep();
    }
}


void LinearSpeedTester::saveTest(float const &lin)
{
    float dt, dx, dy, dth, speed_x, speed_y, speed_th;
    //compute delta
    dx = tf_robot_new_.getOrigin().x() - tf_robot_old_.getOrigin().x();
    dy = tf_robot_new_.getOrigin().y() - tf_robot_old_.getOrigin().y();
    dth = tf::getYaw(tf_robot_new_.getRotation()) - tf::getYaw(tf_robot_old_.getRotation());
    dt = tf_robot_new_.stamp_.toSec() - tf_robot_old_.stamp_.toSec();

    //compute speed
    if(dt == 0.0)
    {
        ROS_ERROR_STREAM("Delta time = 0.0 at linear test : "<< lin <<" m/s.");
        return;
    }
    speed_x = dx/dt;
    speed_y = dy/dt;
    speed_th = dth/dt;

    //add new values to file
    ofilestream_ << lin << "\t" << dt << "\t";
    ofilestream_ << dx << "\t" << dy << "\t" << dth << "\t";
    ofilestream_ << speed_x << "\t" << speed_y << "\t" << speed_th << "\n";
    ROS_INFO_STREAM( "Linear speed test : " << lin
                    << "\tdx = " << dx << "\tdy = " << dy << "\tdth = " << dth
                    << "\tspeed_x = " << speed_x << "\tspeed_y = " << speed_y << "\tspeed_th = " << speed_th);

}

void LinearSpeedTester::performLinearTest()
{
    ROS_INFO_STREAM("Start linear speed test");
    int nb_test =  speed_max_*2.0/speed_step_ + 1.0;
    int i=1;
    for(float lin_speed = - speed_max_; lin_speed <= speed_max_; lin_speed += speed_step_)
    {
        ros::spinOnce();
        if(!ros::ok())
            return;
        ROS_INFO_STREAM("Test #"<<i++<<"/"<<nb_test<<". Linear speed = "<< lin_speed << " m/s");
        testSpeed(lin_speed);
    }
}


/*****************END CLASS DEFINE******************/


int main(int argc, char **argv)
{
    ros::init(argc, argv, "linear_speed_test_node");
    ros::NodeHandle nh;
    LinearSpeedTester lst(nh);

    while(!lst.tf_listener_.waitForTransform("base_link", "odom", ros::Time::now(), ros::Duration(3.0)) && ros::ok())
    {
        ROS_WARN_STREAM("No tf between odom and base link. Waiting for a tf...");
        ros::spinOnce();
    }
    lst.performLinearTest();

    return 0;
}
