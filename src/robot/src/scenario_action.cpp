#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <robot/ScenarioAction.h>

#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <scenario_msgs/Path.h>
#include <scenario_msgs/PathFeedback.h>


#define PI 3.14159265359
#define K_TH 3.0
#define LOOP_RATE 60.0
#define ANGULAR_SPEED_MAX (PI/2.0)
#define INIT_DU 10.0
#define NEXT_POINT_DISTANCE_THRESH 0.10
#define LAST_POINT_DISTANCE_THRESH 0.01
#define LAST_POINT_ANGLE_THRESH (PI/18) // PI/18 rad = 10°
#define RATIO_PUBLISH_RATE (LOOP_RATE/10)

class PathFollower
{
protected:
    ros::NodeHandle nh_;
    geometry_msgs::PoseArray path_;
    int path_uid_;
    //ros::Subscriber path_sub_;
    ros::Publisher cmd_pub_;
    ros::Publisher ratio_pub_;
    tf::TransformListener tf_listener_;
    int index_path_;
    size_t size_path_;
    double du_;
    double dth_;
    int cpt_;
    float linear_speed_;
    double first_du_;

public:
    PathFollower(ros::NodeHandle nh):
        nh_(nh),
        index_path_(0),
        size_path_(0),
        du_(INIT_DU),
        dth_(PI),
        cpt_(0),
        linear_speed_(0.10)
{

     cmd_pub_   = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
     ratio_pub_ = nh_.advertise<scenario_msgs::PathFeedback>("path_feedback", 1);
}
    ~PathFollower(){};

    void pathCB(const scenario_msgs::Path &msg);
    void computeCmd(double &lin, double &ang);
    void computeLastPointAngleCmd(double &lin, double &ang);
    void spinOnce();
    void speedCB(const std_msgs::Float64 &msg);
    void publishRatio();
};



void PathFollower::pathCB(const scenario_msgs::Path &msg)
{
	path_uid_ = msg.uid;
    path_ = msg.path;
    index_path_ = 0;
    size_path_ = path_.poses.size();
    ROS_INFO_STREAM("New path received :   id = " << path_uid_ << "  |  size = " << size_path_ << "  |  goal = "
                    << path_.poses.rbegin()->position.x << " ; " <<path_.poses.rbegin()->position.y);
}


void PathFollower::speedCB(const std_msgs::Float64 &msg)
{
    linear_speed_= msg.data;
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
    if(du_ == INIT_DU)
    {
        first_du_ = du_;
    }
    du_=sqrt(dx*dx+dy*dy);

    alpha = atan2(dy,dx);


    ang = alpha-theta_robot;
    while(ang<-PI)
        ang+=2*PI;
    while(ang>=PI)
        ang-=2*PI;


    ang*=K_TH;
    if(ang>ANGULAR_SPEED_MAX)
    	ang = ANGULAR_SPEED_MAX;
    else if(ang < (- ANGULAR_SPEED_MAX) )
    	ang = -ANGULAR_SPEED_MAX;

    lin=linear_speed_;
}



void PathFollower::computeLastPointAngleCmd(double &lin, double &ang)
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

    double theta_robot;
    double theta_des;

    theta_robot = tf::getYaw(tf_robot.getRotation());
    theta_des = tf::getYaw(path_.poses[index_path_].orientation);
    dth_ = theta_des-theta_robot;
    while(ang<-PI)
        dth_+=2*PI;
    while(ang>=PI)
        dth_-=2*PI;

    ang = dth_;
    ang*=K_TH;
    if(ang>ANGULAR_SPEED_MAX)
        ang = ANGULAR_SPEED_MAX;
    else if(ang < (- ANGULAR_SPEED_MAX) )
        ang = -ANGULAR_SPEED_MAX;

    lin=0.0;
}


void PathFollower::publishRatio()
{
    cpt_++;
        if(cpt_>6)
        {
            double ratio_to_next = (du_ - NEXT_POINT_DISTANCE_THRESH) / (first_du_ - NEXT_POINT_DISTANCE_THRESH);

            scenario_msgs::PathFeedback pathFeedback;
            pathFeedback.uid = path_uid_;
            pathFeedback.ratio = (1.0*index_path_ + (1-ratio_to_next))/size_path_;
            ratio_pub_.publish(pathFeedback);
            cpt_=0;
        }
}



void PathFollower::spinOnce()
{
    geometry_msgs::Twist cmd;
    cmd.linear.y=0;
    cmd.linear.z=0;
    cmd.angular.x=0;
    cmd.angular.y=0;


    if(size_path_ > 0 && index_path_<size_path_)
    {


        if(index_path_ == size_path_-1)
        {
            if(du_ > LAST_POINT_DISTANCE_THRESH)
            {
                computeCmd(cmd.linear.x, cmd.angular.z);
                cmd_pub_.publish(cmd);
            }
            else if(dth_ > LAST_POINT_ANGLE_THRESH)
            {
                computeLastPointAngleCmd(cmd.linear.x, cmd.angular.z);
                cmd_pub_.publish(cmd);
            }
            else
            {
                index_path_++;
                du_=INIT_DU;
                dth_ = PI;
                ROS_INFO_STREAM("Heading to point #"<<index_path_<<"/"<<size_path_<<" : ("
                                        <<path_.poses[index_path_].position.x<<"|"<<path_.poses[index_path_].position.y<<")");
            }
        }
        else
        {
            if( du_ > NEXT_POINT_DISTANCE_THRESH)
            {
                computeCmd(cmd.linear.x, cmd.angular.z);
                cmd_pub_.publish(cmd);
            }
            else
            {
                index_path_++;
                du_=INIT_DU;
                dth_ = PI;
            }
        }
    }
    else if(size_path_ > 0 && index_path_==size_path_)
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
    publishRatio();
}


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
        ros::Subscriber path_sub = nh.subscribe("path", 1, &PathFollower::pathCB, &pf);
        ros::Subscriber speed_sub = nh.subscribe("linear_speed", 1, &PathFollower::speedCB, &pf);
        ros::Rate loop(LOOP_RATE);
        ////////////////////////////////////////////////////

        // check that preempt has not been requested by the client
        while(!as_.isPreemptRequested() && ros::ok())
        {
            pf.spinOnce();
            ros::spinOnce();
            loop.sleep();
        }
        ROS_INFO("%s: Preempted", action_name_.c_str());
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
