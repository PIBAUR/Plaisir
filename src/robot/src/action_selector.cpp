///---INCLUDES---///
/***ROS***/
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
/***Action lib message***/
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <robot/StopAction.h>
#include <robot/ScenarioAction.h>
#include <robot/BatteryAction.h>
#include <robot/PingAction.h>
/***other***/
#include <bitset> //for bit display 

///---INCLUDES---///
/***Flags declaration***/
#define PING_FLAG       (1u<<0)
#define BATTERY_FLAG    (1u<<1)
#define STOP_FLAG       (1u<<2)
#define EMERGENCY_FLAG  (1u<<3)
#define NB_FLAGS_ (4)

/***Thresh values for action***/
#define BATTERY_THRESH_LOW  (30) 
#define BATTERY_THRESH_HIGH (90) 
#define PING_THRESH (200)

class ActionSelector{

    private:    
        ///---ATTRIBUTES---///
        /***action client for request***/
        actionlib::SimpleActionClient<robot::StopAction> emergency_stop_ac_;
        actionlib::SimpleActionClient<robot::StopAction> stop_ac_;
        actionlib::SimpleActionClient<robot::PingAction> ping_ac_;
        actionlib::SimpleActionClient<robot::BatteryAction> battery_ac_;
        actionlib::SimpleActionClient<robot::ScenarioAction> scenario_ac_;
        /***process***/
        ros::Rate loop_rate_;
        unsigned char interrupt_; //active flags values
        unsigned char stop_msg_;
        
        ///---METHODS---///
        void stop_all_actions();
        void maj_stop_flag();

    public:
        ///---CONSTRUCTOR---///
        ActionSelector();
        ~ActionSelector();
        
        ///---CALLBACKS---///
        void bumpersCB(const std_msgs::Bool& msg);
        void batteryCB(const std_msgs::Int32& msg);
        void pingCB(const std_msgs::Int32& msg);
        void emergencystopCB(const std_msgs::Bool& msg);
        void front_obstacleCB(const std_msgs::Bool& msg);

        /***main loop***/
        void spin();
};


///---CONSTRUCTOR---///
ActionSelector::ActionSelector():loop_rate_(20),
    emergency_stop_ac_("emergency_stop_action", true),
    stop_ac_("stop_action", true),
    ping_ac_("ping_action", true),
    battery_ac_("low_battery_action", true),
    scenario_ac_("scenario_action", true),
    //sub_bumpers_("bumpers",nh_, boost::bind(&ActionSelector::front_obstacleCB, this, _1)),
    //sub_battery_("battery",nh_, boost::bind(&ActionSelector::front_obstacleCB, this, _1)),
    //sub_ping_("ping",nh_,boost::bind(&ActionSelector::front_obstacleCB, this, _1)),
    //sub_emergency_stop_("emergency_stop",nh_, boost::bind(&ActionSelector::front_obstacleCB, this, _1)),
    //sub_front_obstacle_("front_obstacle",nh_, boost::bind(&ActionSelector::front_obstacleCB, this, _1)),
    interrupt_(0), stop_msg_(0)
{
    
    /***CREATE CLIENT FOR interrupt_****/
    /***WAITING FOR SERVERS TO START***/

    ROS_INFO("Waiting for action servers to start.");
    emergency_stop_ac_.waitForServer();
    stop_ac_.waitForServer();
    ping_ac_.waitForServer();
    battery_ac_.waitForServer();
    scenario_ac_.waitForServer();
    ROS_INFO("======> All action servers started.");
    
    /***init for getting state : prevent from error message***/

    robot::StopGoal es_goal;
    es_goal.goal = true;
    emergency_stop_ac_.sendGoal(es_goal);
    emergency_stop_ac_.cancelGoal();

    robot::StopGoal s_goal;
    s_goal.goal = true;
    stop_ac_.sendGoal(s_goal);
    stop_ac_.cancelGoal();

    robot::BatteryGoal battery_goal;
    battery_goal.goal = true;
    battery_ac_.sendGoal(battery_goal);
    battery_ac_.cancelGoal();

    robot::PingGoal ping_goal;
    ping_goal.goal = true;
    ping_ac_.sendGoal(ping_goal);
    ping_ac_.cancelGoal();

    robot::ScenarioGoal scenario_goal;
    scenario_goal.goal = true;
    scenario_ac_.sendGoal(scenario_goal);
    scenario_ac_.cancelGoal();

}

ActionSelector::~ActionSelector(){}

///---METHODS---///

/*** CALLBACKS ***/
void ActionSelector::bumpersCB(const std_msgs::Bool& msg)
{
    ROS_INFO("Receive Bumpers =  %s", msg.data?"true":"false");
    if(msg.data)
    {
        //interrupt_ =  interrupt_ | STOP_FLAG;
        stop_msg_ = stop_msg_ | (1u<<0);
    }   
    else
    {
        //interrupt_ =  interrupt_ & ~STOP_FLAG;
        stop_msg_ = stop_msg_ & ~(1u<<0);
    }
    maj_stop_flag();
}

void ActionSelector::batteryCB(const std_msgs::Int32& msg)
{
    ROS_INFO("Receive Battery =  %d", msg.data);
    if(msg.data<BATTERY_THRESH_LOW)
        interrupt_ =  interrupt_ | BATTERY_FLAG;
    else if(msg.data>BATTERY_THRESH_HIGH)
    {
        interrupt_ =  interrupt_ & ~BATTERY_FLAG;
        if(battery_ac_.getState()==actionlib::SimpleClientGoalState::ACTIVE)
            battery_ac_.cancelGoal();
    }
}

void ActionSelector::pingCB(const std_msgs::Int32& msg)
{
    ROS_INFO("Receive Ping =  %d", msg.data);
    if(msg.data>PING_THRESH)
        interrupt_ =  interrupt_ | PING_FLAG;
    else
    {
        interrupt_ =  interrupt_ & ~PING_FLAG;
        if(ping_ac_.getState()==actionlib::SimpleClientGoalState::ACTIVE)
            ping_ac_.cancelGoal();
    }
}

void ActionSelector::emergencystopCB(const std_msgs::Bool& msg)
{
    ROS_INFO("Receive EmergencyStop =  %s ", msg.data?"true":"false");
    if(msg.data)
        interrupt_ =  interrupt_ | EMERGENCY_FLAG;
    else
        interrupt_ =  interrupt_ & ~EMERGENCY_FLAG;
}

void ActionSelector::front_obstacleCB(const std_msgs::Bool& msg)
{
    ROS_INFO("Receive Front Obstacle =  %s", msg.data?"true":"false");
    if(msg.data)
    {
        //interrupt_ =  interrupt_ | STOP_FLAG;
        stop_msg_ = stop_msg_ | (1u<<1);
    }   
    else
    {
        //interrupt_ =  interrupt_ & ~STOP_FLAG;
        stop_msg_ = stop_msg_ & ~(1u<<1);
    }
    maj_stop_flag();
}





void ActionSelector::stop_all_actions()
{
    ROS_INFO("Stoping all action...");
    scenario_ac_.cancelGoal();
    ping_ac_.cancelGoal();
    emergency_stop_ac_.cancelGoal();
    stop_ac_.cancelGoal();
    battery_ac_.cancelGoal();
    ROS_INFO("Done.");
}





void ActionSelector::maj_stop_flag()
{
    if(stop_msg_)
        interrupt_ =  interrupt_ | STOP_FLAG;
    else
        interrupt_ =  interrupt_ & ~STOP_FLAG;
}








/***main loop***/
void ActionSelector::spin()
{
    std::bitset<NB_FLAGS_> b_interrupt; //for bit display
    while(ros::ok()){
        b_interrupt=interrupt_;
        ROS_INFO("interrupt value : %s", b_interrupt.to_string().c_str());

        /**Emergency process**/
        if(interrupt_ & EMERGENCY_FLAG)
        {
            if(emergency_stop_ac_.getState()!=actionlib::SimpleClientGoalState::ACTIVE)
            {
                stop_all_actions();
                
                ROS_INFO("Start request send to Emergensy Stop Action.");
                robot::StopGoal goal;
                goal.goal = true;
                emergency_stop_ac_.sendGoal(goal);
            }
        }
        else if( !(interrupt_ & STOP_FLAG) 
        && (emergency_stop_ac_.getState()==actionlib::SimpleClientGoalState::ACTIVE))
        {
            //bool es_finish = emergency_stop_ac.waitForResult(ros::Duration(1.0));
            ROS_INFO("Canceling Emergensy Stop action.");
            emergency_stop_ac_.cancelGoal();
        }
        
        /**stop process**/
        else if(interrupt_ & STOP_FLAG)
        {
            if(stop_ac_.getState()!=actionlib::SimpleClientGoalState::ACTIVE)
            {
                stop_all_actions();
                
                ROS_INFO("Start request send to Stop Action.");
                robot::StopGoal goal;
                goal.goal = true;
                stop_ac_.sendGoal(goal);
                
                ROS_INFO("Wait 5 sec...");
                stop_ac_.waitForResult(ros::Duration(1.0));
                ROS_INFO("Canceling Stop action.");
                stop_ac_.cancelGoal();
            }
        }
        
        /**battery process**/
        else if(interrupt_ & BATTERY_FLAG)
        {
            if(battery_ac_.getState()!=actionlib::SimpleClientGoalState::ACTIVE)
            {
                stop_all_actions();
                robot::BatteryGoal goal;
                goal.goal = true;
                ROS_INFO("Start request send to battery Action.");
                battery_ac_.sendGoal(goal);
            }
        }
        
        /**ping process**/
        else if(interrupt_ & PING_FLAG)
        {
            if(ping_ac_.getState()!=actionlib::SimpleClientGoalState::ACTIVE)
            {
                stop_all_actions();
                robot::PingGoal goal;
                goal.goal = true;
                ROS_INFO("Start request send to ping Action.");
                ping_ac_.sendGoal(goal);
            }
        }
        
        /**scenario process**/
        else if (interrupt_ == 0)
        {
            if(scenario_ac_.getState()!=actionlib::SimpleClientGoalState::ACTIVE)
            {
                
                stop_all_actions();
                robot::ScenarioGoal goal;
                goal.goal = true;
                ROS_INFO("Start request send to scenario Action.");
                scenario_ac_.sendGoal(goal);
            }
        }
        else
        {
            ROS_WARN("Something went wrong.");
        }
        ros::spinOnce();
        loop_rate_.sleep();
    }
}







int main (int argc, char **argv)
{
    ros::init(argc, argv, "action_selector");
    ros::NodeHandle nh;
    ActionSelector action_selector;
    ros::Subscriber sub_bumpers = nh.subscribe("bumpers", 1, &ActionSelector::bumpersCB, &action_selector);
    ros::Subscriber sub_battery = nh.subscribe("battery", 1, &ActionSelector::batteryCB, &action_selector);
    ros::Subscriber sub_ping = nh.subscribe("ping", 1, &ActionSelector::pingCB, &action_selector);
    ros::Subscriber sub_emergency_stop = nh.subscribe("emergency_stop", 1, &ActionSelector::emergencystopCB, &action_selector);
    ros::Subscriber sub_front_obstacle = nh.subscribe("front_obstacle", 1, &ActionSelector::front_obstacleCB, &action_selector);
    action_selector.spin();
    ros::spin();
    return 0;
}


