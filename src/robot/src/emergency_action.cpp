#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <robot/StopAction.h>

class EmergencyStopAction
{
protected:

    ros::NodeHandle nh_;
    //NodeHandle instance must be created before this line. Otherwise strange error may occur.
    actionlib::SimpleActionServer<robot::StopAction> as_; 
    std::string action_name_;


public:

    EmergencyStopAction(std::string name) :
        as_(nh_, name, boost::bind(&EmergencyStopAction::executeCB, this, _1), false),
        action_name_(name)
    {
        as_.start();
    }

    ~EmergencyStopAction(void)
    {
    }
    

    void executeCB(const robot::StopGoalConstPtr &goal)
    {
        // helper variables
        ros::Rate r(1);
        ROS_INFO("%s: Start.", action_name_.c_str());

        // check that preempt has not been requested by the client
        while(!as_.isPreemptRequested() && ros::ok())
        {
            ROS_INFO("%s: Do Emergency Stop Action STUFF...", action_name_.c_str());
            r.sleep();
        }
        ROS_INFO("%s: Preempted", action_name_.c_str());
        
        // set the action state to preempted
        as_.setPreempted();
    }
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "emergencystop_server");
    EmergencyStopAction emergencyStop(ros::this_node::getName());
    ros::spin();
    return 0;
}
