#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <robot/BatteryAction.h>

class BatteryAction
{
protected:

    ros::NodeHandle nh_;
    //NodeHandle instance must be created before this line. Otherwise strange error may occur.
    actionlib::SimpleActionServer<robot::BatteryAction> as_; 
    std::string action_name_;


public:

    BatteryAction(std::string name) :
        as_(nh_, name, boost::bind(&BatteryAction::executeCB, this, _1), false),
        action_name_(name)
    {
        as_.start();
    }

    ~BatteryAction(void)
    {
    }
    

    void executeCB(const robot::BatteryGoalConstPtr &goal)
    {
        // helper variables
        ros::Rate r(1);
        ROS_INFO("%s: Start.", action_name_.c_str());

        // check that preempt has not been requested by the client
        while(!as_.isPreemptRequested() && ros::ok())
        {
            ROS_INFO("%s: Do Battery Action STUFF...", action_name_.c_str());
            r.sleep();
        }
        ROS_INFO("%s: Preempted", action_name_.c_str());
        
        // set the action state to preempted
        as_.setPreempted();
    }
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "battery_server");
    BatteryAction battery(ros::this_node::getName());
    ros::spin();
    return 0;
}
