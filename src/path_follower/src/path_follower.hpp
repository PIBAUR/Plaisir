#include <ros/ros.h>

#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <scenario_msgs/Path.h>
#include <scenario_msgs/PathFeedback.h>
#include <scenario_msgs/TimeAtPoseArray.h>


#include "sstream"

#define PI 3.14159265359
#define K_TH 3.0
#define LOOP_RATE 60.0
#define ANGULAR_SPEED_MAX (PI/2.0)
#define LINEAR_SPEED_MAX (1.0)
#define INIT_DU 10.0
#define NEXT_POINT_DISTANCE_THRESH 0.10
#define LAST_POINT_DISTANCE_THRESH 0.01
#define LAST_POINT_ANGLE_THRESH (PI/18) // PI/18 rad = 10°
#define RATIO_PUBLISH_RATE_DIVIDOR (6)

class PathFollower
{
protected:
    ros::NodeHandle nh_;

    ros::Publisher cmd_pub_;
    ros::Publisher ratio_pub_;
    tf::TransformListener tf_listener_;
    std::string robot_frame_;

    geometry_msgs::PoseArray path_;
    int path_uid_;
    int index_path_;
    int size_path_;
    scenario_msgs::TimeAtPoseArray time_at_poses_;
    int index_sequence_;
    bool backward_;
    float linear_speed_;
    bool idle_;
    ros::Time end_idle_time_;

    double first_du_;
    double du_;
    double dth_;

    int cpt_;


public:
    PathFollower(ros::NodeHandle nh);
    ~PathFollower(){};

    void pathCB(const scenario_msgs::Path &msg);
    void computeCmd(double &lin, double &ang);
    void computeLastPointAngleCmd(double &lin, double &ang);
    float distanceToGoal(size_t index_goal);
    float distanceBetweenPoints(size_t index1, size_t index2);
    void computeAverageSpeed(size_t index_goal, float time);
    void initNextGoal();
    void publishRatio();
    void spinOnce();
};


