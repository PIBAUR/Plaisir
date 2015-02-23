#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <robot/StopAction.h>
#include <geometry_msgs/Twist.h>

class StopAction
{
protected:

    ros::NodeHandle nh_;
    //NodeHandle instance must be created before this line. Otherwise strange error may occur.
    actionlib::SimpleActionServer<robot::StopAction> as_; 
    std::string action_name_;


public:

    StopAction(std::string name) :
        as_(nh_, name, boost::bind(&StopAction::executeCB, this, _1), false),
        action_name_(name)
    {
        as_.start();
    }

    ~StopAction(void)
    {
    }
    

    void executeCB(const robot::StopGoalConstPtr &goal)
    {
        // helper variables
        ros::Rate r(5);
        ROS_INFO("%s: Start.", action_name_.c_str());
        ros::NodeHandle nh;
        ros::Publisher cmd_pub   = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);

        geometry_msgs::Twist cmd;
        cmd.linear.x=0;
        cmd.linear.y=0;
        cmd.linear.z=0;
        cmd.angular.x=0;
        cmd.angular.y=0;
        cmd.angular.z=0;

        // check that preempt has not been requested by the client
        while(!as_.isPreemptRequested() && ros::ok())
        {
            ROS_INFO("%s: Stopping robot...", action_name_.c_str());
            cmd_pub.publish(cmd);
            r.sleep();
        }
        ROS_INFO("%s: Preempted", action_name_.c_str());
        
        // set the action state to preempted
        as_.setPreempted();
    }
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "stop_server");
    StopAction stop(ros::this_node::getName());
    ros::spin();
    return 0;
}
