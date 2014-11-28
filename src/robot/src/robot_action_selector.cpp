#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <robot/StopAction.h>
#include <robot/ScenarioAction.h>
#include <robot/BatteryAction.h>
#include <robot/PingAction.h>

#include <bitset>



unsigned char interrupt;

void bumpersCB(const std_msgs::Bool& msg)
{
    ROS_INFO("Receive Bumpers =  %s ", msg.data?"true":"false");
    unsigned char level = (1u<<2);
    if(msg.data)
        interrupt =  interrupt | level;
    else
        interrupt =  interrupt & ~level;
}

void batteryCB(const std_msgs::Int32& msg)
{
    ROS_INFO("Receive Battery =  %d", msg.data);
    unsigned char level = (1u<<1);
   if(msg.data<30)
        interrupt =  interrupt | level;
    else
        interrupt =  interrupt & ~level;
}

void pingCB(const std_msgs::Int32& msg)
{
    ROS_INFO("Receive Ping =  %d", msg.data);
    unsigned char level = (1u<<0);
    if(msg.data>200)
        interrupt =  interrupt | level;
    else
        interrupt =  interrupt & ~level;
}

void emergencystopCB(const std_msgs::Bool& msg)
{
    ROS_INFO("Receive EmergencyStop =  %s ", msg.data?"true":"false");
    unsigned char level = (1u<<3);
    if(msg.data)
        interrupt =  interrupt | level;
    else
        interrupt =  interrupt & ~level;
}

void front_obstacleCB(const std_msgs::Bool& msg)
{
    ROS_INFO("Receive Front Obstacle =  %s ", msg.data?"true":"false");
    unsigned char level = (1u<<2);
    if(msg.data)
        interrupt =  interrupt | level;
    else
        interrupt =  interrupt & ~level;
}





int main (int argc, char **argv)
{
    ros::init(argc, argv, "action_selector");
    ros::NodeHandle nh;
    /***CREATE CLIENT FOR INTERRUPT****/
    /***WAITING FOR SERVERS TO START***/
    // create the action client
    // true causes the client to spin its own thread
    //actionlib::SimpleActionClient<robot::TypeAction> ac("serveur_name", true);
    
    ROS_INFO("Waiting for action servers to start.");
    
    actionlib::SimpleActionClient<robot::StopAction> emergency_stop_ac("emergencystop_server", true);
    emergency_stop_ac.waitForServer(); //will wait for infinite time
    ROS_INFO("Emergency Stop Action server started.");
    
    actionlib::SimpleActionClient<robot::StopAction> stop_ac("stop_server", true);
    stop_ac.waitForServer();
    ROS_INFO("Stop Action server started.");
    
    actionlib::SimpleActionClient<robot::PingAction> ping_ac("ping_server", true);
    ping_ac.waitForServer();
    ROS_INFO(" Ping Action server started.");
    
    actionlib::SimpleActionClient<robot::BatteryAction> battery_ac("battery_server", true);
    battery_ac.waitForServer();
    ROS_INFO("Battery Action server started.");
    
    actionlib::SimpleActionClient<robot::ScenarioAction> scenario_ac("scenario_server", true);
    scenario_ac.waitForServer();
    ROS_INFO("Scenario Action server started.");
    
    ROS_INFO("======> All action servers started.");
    
    
    
    ros::Subscriber bumpers = nh.subscribe("/bumpers",1,bumpersCB);
    ros::Subscriber battery = nh.subscribe("/battery",1,batteryCB);
    ros::Subscriber ping = nh.subscribe("/ping",1,pingCB);
    ros::Subscriber emergency_stop = nh.subscribe("/emergency_stop",1,emergencystopCB);
    ros::Subscriber front_obstacle = nh.subscribe("/front_ostacle",1,front_obstacleCB);
    
    ros::Rate loop(3);
    interrupt = 0;
    
    while(ros::ok()){
        std::bitset<4> b_interrupt = interrupt;
        ROS_INFO("Interrupt value : %s", b_interrupt.to_string().c_str());
        try 
        {
            ROS_INFO(" %s ", emergency_stop_ac.getState().toString().c_str());
            //if(emergency_stop_ac.getState()==actionlib::SimpleClientGoalState::ACTIVE)
            ROS_INFO("ACTIVE <3 <3 <3 <3 <3 <3 <3 <3");
        }
        catch(int& e)
        {
            ROS_INFO("NO ACTIVE </3 </3 </3 <3 <3 <3 <3 <3");
        }
        if(interrupt & (1u<<3))
        {
            robot::StopGoal es_goal;
            
            es_goal.goal = true;
            ROS_INFO("Start request send to Emergensy Stop Action.");
            emergency_stop_ac.sendGoal(es_goal);
            ROS_INFO("Wait 3 sec...");
            bool es_finish = emergency_stop_ac.waitForResult(ros::Duration(1.0));
            ROS_INFO("Canceling Emergensy Stop action.");
            emergency_stop_ac.cancelGoal();
        }
        else if(interrupt & (1u<<2))
        {
            robot::StopGoal s_goal;
            s_goal.goal = true;
            ROS_INFO("Start request send to Stop Action.");
            stop_ac.sendGoal(s_goal);
            ROS_INFO("Wait 3 sec...");
            bool s_finish = stop_ac.waitForResult(ros::Duration(1.0));
            ROS_INFO("Canceling Stop action.");
            stop_ac.cancelGoal();
            
        }
        else if(interrupt & (1u<<1))
        {
             robot::BatteryGoal battery_goal;
            battery_goal.goal = true;
            ROS_INFO("Start request send to battery Action.");
            battery_ac.sendGoal(battery_goal);
            ROS_INFO("Wait 3 sec...");
            bool b_finish = battery_ac.waitForResult(ros::Duration(1.0));
            ROS_INFO("Canceling action.");
            battery_ac.cancelGoal();
        }
        else if(interrupt & (1u<<0))
        {
            robot::PingGoal ping_goal;
            ping_goal.goal = true;
            ROS_INFO("Start request send to ping Action.");
            ping_ac.sendGoal(ping_goal);
            ROS_INFO("Wait 3 sec...");
            bool p_finish = ping_ac.waitForResult(ros::Duration(1.0));
            ROS_INFO("Canceling action.");
            ping_ac.cancelGoal();
        }
        else if (interrupt == 0)
        {
            robot::ScenarioGoal scenario_goal;
            scenario_goal.goal = true;
            ROS_INFO("Start request send to scenario Action.");
            scenario_ac.sendGoal(scenario_goal);
            ROS_INFO("Wait 3 sec...");
            bool sc_finish = scenario_ac.waitForResult(ros::Duration(1.0));
            ROS_INFO("Canceling action.");
            scenario_ac.cancelGoal();
        }
        else
        {
            ROS_WARN("Something went wrong.");
        }
        ros::spinOnce();
        loop.sleep();
    }
    
    // send a goal to the action
    
    /**
     * robot::""Goal ""goal;
     * goal.start = true;
     * ROS_INFO("Start request send to "" Action.");
     * ac.sendGoal(es_goal);
     * ROS_INFO("Wait 3 sec...");
     * bool finish = ac.waitForResult(ros::Duration(3.0));
     * ROS_INFO("Canceling action.");
     * ac.cancelGoal();
    **/
    
        



    //exit
    return 0;
}

