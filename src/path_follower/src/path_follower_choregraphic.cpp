///---ROS---////
#include <ros/ros.h>
#include <scenario_msgs/PathChoregraphic.h>
#include <scenario_msgs/TwistStampedArray.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Twist.h>
#include <scenario_msgs/PathFeedback.h>


///---CONSTANTS---///
#define PI 3.14159265359
#define LOOP_RATE 100.0
#define ANGULAR_SPEED_MAX (PI)
#define LINEAR_SPEED_MAX (1.0)
#define RATIO_PUBLISH_RATE_DIVIDOR (6.0)

class ChoregrahicPathFollower
{
protected:
    ros::NodeHandle nh_;

    ros::Publisher cmd_pub_;
    ros::Publisher ratio_pub_;

    scenario_msgs::TwistStampedArray twist_array_;
    int path_uid_;
    int index_path_;
    int size_path_;

    ros::Time next_twist_time_;

    float linear_speed_;
    float angular_speed_;

    int cpt_;


public:
    ChoregrahicPathFollower(ros::NodeHandle nh);
    ~ChoregrahicPathFollower(){};

    void pathCB(const scenario_msgs::PathChoregraphic &msg);
    void getNewTwist();
    void publishRatio();
    void spinOnce();
};





ChoregrahicPathFollower::ChoregrahicPathFollower(ros::NodeHandle nh):
    nh_(nh),
    index_path_(0),
    size_path_(-1),
    cpt_(0),
    linear_speed_(0.0),
    angular_speed_(0.0)
{
    cmd_pub_   = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    ratio_pub_ = nh_.advertise<scenario_msgs::PathFeedback>("path_feedback", 1);
}




void ChoregrahicPathFollower::pathCB(const scenario_msgs::PathChoregraphic &msg)
{

    path_uid_ = msg.uid;
    twist_array_ = msg.path;
    index_path_ = 0;
    size_path_ = twist_array_.twists.size();
    next_twist_time_.sec = msg.start_timestamp.sec;
    next_twist_time_.nsec = msg.start_timestamp.nsec;
    linear_speed_ = 0.0;
    angular_speed_ = 0.0;
    cpt_ = 0;

    ROS_INFO_STREAM("New path received :   id = " << path_uid_ << "  |  size = " << size_path_ );
}




void ChoregrahicPathFollower::getNewTwist()
{

    if(index_path_ > size_path_)
    {
        linear_speed_ = 0.0;
        angular_speed_ = 0.0;
        ROS_INFO("End of path");
        return;
    }

    linear_speed_ = twist_array_.twists[index_path_].twist.linear.x;
    angular_speed_ = twist_array_.twists[index_path_].twist.angular.z;
    next_twist_time_.nsec = twist_array_.twists[index_path_].header.stamp.nsec;
    next_twist_time_.sec = twist_array_.twists[index_path_].header.stamp.sec;
    ROS_INFO_STREAM("New speed #"<<index_path_<<"/"<<size_path_<<" :linear = "
                     <<linear_speed_<<" and angular = "<<angular_speed_<<" rad/s for "
                     << (next_twist_time_-ros::Time::now()).sec);

    if(fabs(linear_speed_) > LINEAR_SPEED_MAX/2.0)
		ROS_WARN("Linear speed may be to high...");
    if(fabs(angular_speed_) > ANGULAR_SPEED_MAX/2.0)
        ROS_WARN("Linear speed may be to high...");

    index_path_++;
}



void ChoregrahicPathFollower::publishRatio()
{
	scenario_msgs::PathFeedback pathFeedback;
	pathFeedback.uid = path_uid_;
	pathFeedback.ratio = (float)index_path_ / (float)size_path_;
	ratio_pub_.publish(pathFeedback);
}




void ChoregrahicPathFollower::spinOnce()
{
	geometry_msgs::Twist cmd;
	cmd.linear.x=0.0;
	cmd.linear.y=0.0;
    cmd.linear.z=0.0;
    cmd.angular.x=0.0;
    cmd.angular.y=0.0;
    cmd.angular.z=0.0;

    if(size_path_ > 0 && index_path_<size_path_)
    {
        //while(next_twist_time_ < ros::Time::now() && index_path_<size_path_ && ros::ok())
        if(next_twist_time_ < ros::Time::now())
            getNewTwist();
        cmd.linear.x=linear_speed_;
        cmd.angular.z=angular_speed_;
        cmd_pub_.publish(cmd);
    }
    else if(size_path_ > 0 && index_path_>=size_path_)
    {
        cmd_pub_.publish(cmd);
        size_path_=-1;
		ROS_INFO_STREAM("Path follower ended");
    }
    else if(size_path_ ==0)
    {
        cmd_pub_.publish(cmd);
        size_path_=-1;
		ROS_INFO_STREAM("Path follower received 0 sized path");
    }

    // publish ratio
	if(cpt_>RATIO_PUBLISH_RATE_DIVIDOR)
	{
		publishRatio();
		cpt_=0;
	}
    cpt_++;
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "path_follower_choregraphic_node");
	ros::NodeHandle nh;
	ChoregrahicPathFollower cpf(nh);
	ros::Subscriber path_sub = nh.subscribe("twist_path", 1, &ChoregrahicPathFollower::pathCB, &cpf);
	ros::Rate loop(LOOP_RATE);

	while(ros::ok())
	{
		cpf.spinOnce();
		ros::spinOnce();
		loop.sleep();
	}

    return 0;
}
