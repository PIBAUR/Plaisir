#include <ros/ros.h>

// messages
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/GetMap.h>
#include <nav_msgs/MapMetaData.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Pose2D.h>

//service
#include <path_checker/PathCheckerReq.h>
#include <nav_msgs/GetMap.h>

//lib opencv
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>

#include <sstream>

//CONSTANT
#define PI 3.14159265358979323846
#define SHRINK_DEFAULT_STEP     0.10
#define SHRINK_DEFAULT_LIMIT    0.80
#define ROTATE_DEFAULT_STEP     (PI/6) // 30°
#define ROTATE_DEFAULT_LIMIT    (2*PI) // 360 °


class PathChecker
{
protected:
    ros::NodeHandle nh_;

    cv::Mat_<bool> map_;
    double map_resolution_;
    int x_origin_map_pixel_;
    int y_origin_map_pixel_;
    double x_origin_map_meter_;
    double y_origin_map_meter_;
    geometry_msgs::Pose2D pose_target_;

    double robot_size_;

    geometry_msgs::PoseArray path_working_;
    geometry_msgs::PoseArray path_of_pose_;
    cv::Point_<int> path_of_point_[];

public:
    PathChecker(ros::NodeHandle nh);
    ~PathChecker(){};

    void poseToPoint(const geometry_msgs::Pose &pose, cv::Point_<int> &point);
    void pointToPose(const cv::Point_<int> &point, geometry_msgs::Pose &pose);
    void occupancyGridToMat(const nav_msgs::OccupancyGrid &occupancy_grid);
    void poseFromTarget(geometry_msgs::Pose &pose);

    bool isPathWayFree(geometry_msgs::PoseArray const &path);

    bool isWayFree(const cv::Point_<int> &point1, const cv::Point_<int> &point2);

    bool serviceCB(path_checker::PathCheckerReq::Request &req, path_checker::PathCheckerReq::Response &res);

    void shrinkPath(const float coef);
    void rotatePath(const float angle);
};
