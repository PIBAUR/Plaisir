#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <robot/ScenarioAction.h>



///TEST////////////////////////////////////////////
///TEST////////////////////////////////////////////
///TEST////////////////////////////////////////////

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseArray.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <std_msgs/Float64.h>


#define PI 3.14159265359
#define K_TH 5.0
#define LOOP_RATE 60

class PathFollower
{
protected:
    ros::NodeHandle nh_;
    geometry_msgs::PoseArray path_;
    //ros::Subscriber path_sub_;
    ros::Publisher cmd_pub_;
    ros::Publisher ratio_pub_;
    tf::TransformListener tf_listener_;
    int index_path_;
    size_t size_path_;
    double du_;
    int cpt_;

public:
    PathFollower(ros::NodeHandle nh):
        nh_(nh),
        index_path_(0),
        size_path_(0),
        du_(10.0),
        cpt_(0)
{

     cmd_pub_   = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
     ratio_pub_ = nh_.advertise<std_msgs::Float64>("path_feedback", 1);
}
    ~PathFollower(){};

    void pathCB(const geometry_msgs::PoseArray &msg);
    void computeCmd(double &lin, double &ang);
    void spinOnce();
};



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
    else if(size_path_ !=0 && index_path_==size_path_)
    {
        cmd.linear.x = 0;
        cmd.angular.z = 0;
        cmd_pub_.publish(cmd);
    }
    else if(size_path_ ==0)
    {
        cmd.linear.x = 0;
        cmd.angular.z = 0;
        cmd_pub_.publish(cmd);
        size_path_==-1;
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











///TEST////////////////////////////////////////////
///TEST////////////////////////////////////////////
///TEST////////////////////////////////////////////

class ScenarioAction
{
protected:

    ros::NodeHandle nh_;
    //NodeHandle instance must be created before this line. Otherwise strange error may occur.
    actionlib::SimpleActionServer<robot::ScenarioAction> as_; 
    std::string action_name_;


public:

    ScenarioAction(std::string name) :
        as_(nh_, name, boost::bind(&ScenarioAction::executeCB, this, _1), false),
        action_name_(name)
    {
        as_.start();
    }

    ~ScenarioAction(void)
    {
    }
    

    void executeCB(const robot::ScenarioGoalConstPtr &goal)
    {
        // helper variables
        ros::Rate r(1);
        ROS_INFO("%s: Start.", action_name_.c_str());
        ////////////////////////////////////////////////////
        ros::NodeHandle nh;
        PathFollower pf(nh);
        ros::Subscriber sub = nh.subscribe("path", 1, &PathFollower::pathCB, &pf);
        ros::Rate loop(LOOP_RATE);
        ////////////////////////////////////////////////////

        // check that preempt has not been requested by the client
        while(!as_.isPreemptRequested() && ros::ok())
        {
            ROS_INFO("%s: Do Scenario Action STUFF...", action_name_.c_str());
            ////////////////////////////////////////////////////
            ////////////////////////////////////////////////////
            ////////////////////////////////////////////////////
            pf.spinOnce();
            ros::spinOnce();
            loop.sleep();
            ////////////////////////////////////////////////////
            ////////////////////////////////////////////////////
            ////////////////////////////////////////////////////

        }
        ROS_INFO("%s: Preempted", action_name_.c_str());
        
        // set the action state to preempted
        as_.setPreempted();
    }
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "scenario_server");
    ScenarioAction stop(ros::this_node::getName());
    ros::spin();
    return 0;
}
