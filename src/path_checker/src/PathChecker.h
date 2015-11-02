#include <ros/ros.h>

// messages
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/GetMap.h>
#include <nav_msgs/MapMetaData.h>
#include <scenario_msgs/Pose2DArray.h>
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

    scenario_msgs::Pose2DArray path_received_;
    //scenario_msgs::Pose2DArray path_result_;
    scenario_msgs::Pose2DArray path_of_pose_;
    cv::Point_<int> path_of_point_[];

public:
    PathChecker(ros::NodeHandle nh);
    ~PathChecker(){};

    void poseToPoint(const geometry_msgs::Pose2D &pose, cv::Point_<int> &point);
    void pointToPose(const cv::Point_<int> &point, geometry_msgs::Pose2D &pose);
    void occupancyGridToMat(const nav_msgs::OccupancyGrid &occupancy_grid);
    void poseMapToTarget(geometry_msgs::Pose2D &pose);

    bool isPathWayFree();
    bool isWayFree(const cv::Point_<int> &point1, const cv::Point_<int> &point2);

    bool serviceCB(path_checker::PathCheckerReq::Request &req, path_checker::PathCheckerReq::Response &res);

    void rotatePath(const float coef, scenario_msgs::Pose2DArray &poses);
    void shrinkPath(const float angle);
};
