#include <ros/ros.h>

#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <scenario_msgs/PathTravel.h>
#include <scenario_msgs/PathFeedback.h>
#include <scenario_msgs/TimeAtPoseArray.h>


#include "sstream"

#define PI 3.14159265359
#define K_TH 3.0
#define LOOP_RATE 60.0
#define ANGULAR_SPEED_MAX (PI)
#define ANGLE_THRESH_LOW (PI/10.0)
#define ANGLE_THRESH_HIGH (PI/4.0)
#define LINEAR_SPEED_MAX (1.0)
#define LINEAR_SPEED_MIN (0.01)
#define LINEAR_SPEED_DEFAULT (0.20)
#define INIT_DU 10.0
#define NEXT_POINT_DISTANCE_THRESH 0.05
#define LAST_POINT_DISTANCE_THRESH 0.05
#define LAST_POINT_ANGLE_THRESH (PI/36.0) // PI/36 rad = 5°
#define RATIO_PUBLISH_RATE_DIVIDOR (6.0)

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
    float average_linear_speed_;
    bool idle_;
    ros::Time end_idle_time_;
    ros::Time goal_time_;

    double first_du_;
    double du_;
    double dth_;

    int cpt_;


public:
    PathFollower(ros::NodeHandle nh);
    ~PathFollower(){};

    void pathCB(const scenario_msgs::PathTravel &msg);
    void computeCmd(double &lin, double &ang);
    void computeLastPointAngleCmd(double &lin, double &ang);
    float distanceToGoal(size_t index_goal);
    float distanceBetweenPoints(size_t index1, size_t index2);
    void computeAverageSpeed(size_t index_goal, float time);
    void initNextGoal();
    void publishRatio();
    void spinOnce();
};


