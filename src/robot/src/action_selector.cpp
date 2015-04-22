///---INCLUDES---///
/***ROS***/
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
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
        
        ///---PROPERTIES---///
    	bool bumper_state_;
    	int battery_state_;
    	int ping_state_;
    	bool emergency_stop_state_;
    	bool frozen_state_;
    	bool front_obstacle_state_;

    	bool already_stopped_;

    	///---PUBLISHER---///
    	ros::NodeHandle nh_;
    	ros::Publisher state_publisher_;
    	std_msgs::String state_msg;

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
        void freezeCB(const std_msgs::Bool& msg);
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

	bumper_state_(false),
	battery_state_(-1),
	ping_state_(-1),
	emergency_stop_state_(false),
	frozen_state_(false),
	front_obstacle_state_(false),

	already_stopped_(false),

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

    state_publisher_ = nh_.advertise<std_msgs::String>("state", 1);
}

ActionSelector::~ActionSelector(){}

///---METHODS---///

/*** CALLBACKS ***/
void ActionSelector::bumpersCB(const std_msgs::Bool& msg)
{
    if (msg.data != bumper_state_)
    {
    	ROS_INFO("Receive Bumpers =  %s", msg.data?"true":"false");
    	bumper_state_ = msg.data;
    }
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
	if (msg.data != battery_state_)
	{
		ROS_INFO("Receive Battery =  %d", msg.data);
		battery_state_ = msg.data;
	}
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
	if (msg.data != ping_state_)
	{
		ROS_INFO("Receive Ping =  %d", msg.data);
		ping_state_ = msg.data;
	}
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
	if (msg.data != emergency_stop_state_)
	{
		ROS_INFO("Receive EmergencyStop =  %s ", msg.data?"true":"false");
		emergency_stop_state_ = msg.data;
	}
    if(msg.data)
        interrupt_ =  interrupt_ | EMERGENCY_FLAG;
    else
        interrupt_ =  interrupt_ & ~EMERGENCY_FLAG;
}

void ActionSelector::freezeCB(const std_msgs::Bool& msg)
{
	if (msg.data != frozen_state_)
	{
		ROS_INFO("Receive Freeze =  %s ", msg.data?"true":"false");
		frozen_state_ = msg.data;
	}
    if(msg.data)
    {
        stop_msg_ = stop_msg_ | (1u<<2);
    }
    else
    {
        stop_msg_ = stop_msg_ & ~(1u<<2);
    }
    maj_stop_flag();
}

void ActionSelector::front_obstacleCB(const std_msgs::Bool& msg)
{
	if (msg.data != front_obstacle_state_)
	{
		ROS_INFO("Receive Front Obstacle =  %s", msg.data?"true":"false");
		front_obstacle_state_ = msg.data;
	}
    if(msg.data)
    {
        stop_msg_ = stop_msg_ | (1u<<1);
    }   
    else
    {
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



        /**Emergency process**/
        if(interrupt_ & EMERGENCY_FLAG)
        {
            if(emergency_stop_ac_.getState()!=actionlib::SimpleClientGoalState::ACTIVE)
            {
                stop_all_actions();
                
                ROS_INFO("Start request send to Emergency Stop Action.");
                robot::StopGoal goal;
                goal.goal = true;
                emergency_stop_ac_.sendGoal(goal);
				state_msg.data = "EMERGENCY_STOP";
            }
        }
        else if( !(interrupt_ & STOP_FLAG) 
        && (emergency_stop_ac_.getState()==actionlib::SimpleClientGoalState::ACTIVE))
        {
            //bool es_finish = emergency_stop_ac.waitForResult(ros::Duration(1.0));
            ROS_INFO("Canceling Emergency Stop action.");
            emergency_stop_ac_.cancelGoal();
        }
        
        /**stop process (only the first time)**/
        else if(interrupt_ & STOP_FLAG)
        {
            if(stop_ac_.getState()!=actionlib::SimpleClientGoalState::ACTIVE)
            {
            	if (! already_stopped_)
            	{
					stop_all_actions();

					if (! frozen_state_)
					{
						already_stopped_ = true;

						ROS_INFO("Start request send to Stop Action.");
						robot::StopGoal goal;
						goal.goal = true;

						stop_ac_.sendGoal(goal);
						state_msg.data = "STOP";

						//TODO: remove the timer
						ROS_INFO("Wait 0,01 sec...");
						stop_ac_.waitForResult(ros::Duration(0.01));

						ROS_INFO("Canceling Stop action.");
						stop_ac_.cancelGoal();

						robot::ScenarioGoal scenarioGoal;
						scenarioGoal.goal = true;
						ROS_INFO("Start request send to scenario Action in stop mode.");
						scenario_ac_.sendGoal(scenarioGoal);
					}
            	}
            }

            if (frozen_state_)
            {
            	frozen_state_ = false;
                stop_msg_ = stop_msg_ & ~(1u<<2);
				maj_stop_flag();
				//TODO: give a feedback, to relaunch after from the server
            }
        }
        
        /**battery process**/
        else if(interrupt_ & BATTERY_FLAG)
        {
            if(battery_ac_.getState()!=actionlib::SimpleClientGoalState::ACTIVE)
            {
                robot::BatteryGoal goal;
                goal.goal = true;
                ROS_INFO("Start request send to battery Action.");
                battery_ac_.sendGoal(goal);
				state_msg.data = "LOW_BATTERY";
            }
        }

        /**ping process**/
        else if(interrupt_ & PING_FLAG)
        {
            if(ping_ac_.getState()!=actionlib::SimpleClientGoalState::ACTIVE)
            {
                robot::PingGoal goal;
                goal.goal = true;
                ROS_INFO("Start request send to ping Action.");
                ping_ac_.sendGoal(goal);
				state_msg.data = "HIGH_PING";
            }
        }

        /**scenario process**/
        else if (interrupt_ == 0)
        {
			state_msg.data = "ACTIVE";
			already_stopped_ = false;

            if(scenario_ac_.getState()!=actionlib::SimpleClientGoalState::ACTIVE)
            {
                stop_all_actions();
                robot::ScenarioGoal goal;
                goal.goal = true;
                ROS_INFO("Start request send to scenario Action.");
                scenario_ac_.sendGoal(goal);
            }

			if(!(interrupt_ & BATTERY_FLAG))
			{
				if(battery_ac_.getState()==actionlib::SimpleClientGoalState::ACTIVE)
				{
					battery_ac_.cancelGoal();
				}

			}

			if(!(interrupt_ & PING_FLAG))
			{
				if(ping_ac_.getState()==actionlib::SimpleClientGoalState::ACTIVE)
				{
					ping_ac_.cancelGoal();
				}
			}
        }
        else
        {
            ROS_WARN("Something went wrong.");
        }

        state_publisher_.publish(state_msg);

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
    ros::Subscriber sub_freeze = nh.subscribe("freeze", 1, &ActionSelector::freezeCB, &action_selector);
    ros::Subscriber sub_front_obstacle = nh.subscribe("front_obstacle", 1, &ActionSelector::front_obstacleCB, &action_selector);
    action_selector.spin();
    ros::spin();
    return 0;
}


