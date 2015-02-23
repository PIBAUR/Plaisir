#include <ros/ros.h>
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


