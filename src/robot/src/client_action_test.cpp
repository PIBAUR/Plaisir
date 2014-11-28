#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <robot/FibonacciAction.h>
#include <robot/StopAction.h>

int main (int argc, char **argv)
{
  ros::init(argc, argv, "test_fibonacci");

  // create the action client
  // true causes the client to spin its own thread
  actionlib::SimpleActionClient<robot::FibonacciAction> ac("fibonacci", true);

  ROS_INFO("Waiting for action server to start.");
  // wait for the action server to start
  ac.waitForServer(); //will wait for infinite time

  ROS_INFO("Action server started, sending goal.");
  // send a goal to the action
  robot::FibonacciGoal goal;
  goal.order = 20;
  ac.sendGoal(goal);

  //wait for the action to return
  bool finished_before_timeout = ac.waitForResult(ros::Duration(3.0));

  if (finished_before_timeout)
  {
    actionlib::SimpleClientGoalState state = ac.getState();
    ROS_INFO("Action finished: %s",state.toString().c_str());
  }
  else
  {
    ROS_INFO("Action did not finish before the time out.");
    ac.cancelGoal();
    
  }
  
  
  
  
  actionlib::SimpleActionClient<robot::StopAction> es_ac("emergencystop", true);
  ROS_INFO("Waiting for action server to start.");
  es_ac.waitForServer(); //will wait for infinite time

  ROS_INFO("Action server started, sending goal.");

  robot::StopGoal es_goal;
  es_goal.start = true;
  es_ac.sendGoal(es_goal);

  //wait for the action to return
  bool es_finished_before_timeout = es_ac.waitForResult(ros::Duration(3.0));

  if (finished_before_timeout)
  {
    actionlib::SimpleClientGoalState state = es_ac.getState();
    ROS_INFO("Action finished: %s",state.toString().c_str());
  }
  else
  {
    ROS_INFO("Action did not finish before the time out.");
    es_ac.cancelGoal();
    
  }
  
  
  
  

  //exit
  return 0;
}
