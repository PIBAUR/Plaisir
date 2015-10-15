#include <ros/ros.h>

#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <scenario_msgs/Path.h>
#include <scenario_msgs/PathFeedback.h>

#include "sstream"

#define PI 3.14159265359
#define K_TH 3.0
#define LOOP_RATE 60.0
#define ANGULAR_SPEED_MAX (PI/2.0)
#define LINEAR_SPEED_MAX (0.20)
#define INIT_DU 10.0
#define NEXT_POINT_DISTANCE_THRESH 0.10
#define LAST_POINT_DISTANCE_THRESH 0.01
#define LAST_POINT_ANGLE_THRESH (PI/18) // PI/18 rad = 10°
#define RATIO_PUBLISH_RATE_DIVIDOR (6)

class PathFollower
{
protected:
    ros::NodeHandle nh_;
    geometry_msgs::PoseArray path_;
    int path_uid_;
    ros::Publisher cmd_pub_;
    ros::Publisher ratio_pub_;
    tf::TransformListener tf_listener_;
    int index_path_;
    int size_path_;
    double du_;
    double dth_;
    int cpt_;
    float linear_speed_;
    double first_du_;
    bool reversed_;
    std::string robot_frame_;


public:
    PathFollower(ros::NodeHandle nh):
        nh_(nh),
        index_path_(0),
        size_path_(-1),
        du_(INIT_DU),
        dth_(PI),
        cpt_(0),
        linear_speed_(0.10),
        reversed_(false)
    {
        cmd_pub_   = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
        ratio_pub_ = nh_.advertise<scenario_msgs::PathFeedback>("path_feedback", 1);

        std::string tf_prefix;
        std::stringstream frame;

        if (nh_.getParam("tf_prefix", tf_prefix))
        {
            frame << tf_prefix <<"/base_link";
        }
        else
        {
            frame<<"/base_link";
        }
        robot_frame_ = frame.str();
        ROS_INFO_STREAM("frame robot in path follower : "<<frame);
    }
    ~PathFollower(){};

    void pathCB(const scenario_msgs::Path &msg);
    void computeCmd(double &lin, double &ang);
    void computeLastPointAngleCmd(double &lin, double &ang);
    void spinOnce();
    void speedCB(const std_msgs::Float64 &msg);
    void reversedCB(const std_msgs::Bool &msg);
    void publishRatio();
};


