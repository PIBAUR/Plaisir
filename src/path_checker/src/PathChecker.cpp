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
    void poseFromTarget(geometry_msgs::Pose2D &pose);

    bool isPathWayFree(scenario_msgs::Pose2DArray const &path2D);

    bool isWayFree(const cv::Point_<int> &point1, const cv::Point_<int> &point2);

    bool serviceCB(path_checker::PathCheckerReq::Request &req, path_checker::PathCheckerReq::Response &res);

    void shrinkPath(const float coef);
    void rotatePath(const float angle);
};



PathChecker::PathChecker(ros::NodeHandle nh):
    nh_(nh),
    map_resolution_ (0.0),
    x_origin_map_pixel_(0), y_origin_map_pixel_(0),
    x_origin_map_meter_(0.0), y_origin_map_meter_(0.0),
    robot_size_(0.0)
{
    ros::spinOnce();

    // calling map service
    ros::ServiceClient map_client = nh_.serviceClient<nav_msgs::GetMap>("static_map");
    nav_msgs::GetMap map_service;
    ROS_INFO_STREAM("Sending map request...");
    bool srv_call_success = false;
    while(!srv_call_success && ros::ok())
    {
        ros::spinOnce();
        if (map_client.call(map_service))
        {
            ROS_INFO_STREAM("Map received : size = " << map_service.response.map.info.width
                            << " x " << map_service.response.map.info.height << " ; resolution = "
                            << map_service.response.map.info.resolution << " m/px.");
            srv_call_success = true;
        }
        else
        {
            ROS_ERROR("Failed to call service GetMap. Waiting 5 sec before retry.");
            ros::Duration(5.0).sleep();
        }
    }
    occupancyGridToMat(map_service.response.map);
}



void PathChecker::poseToPoint(const geometry_msgs::Pose2D &pose, cv::Point_<int> &point)
{
    point.x = -(x_origin_map_meter_ - pose.x)/map_resolution_;
    point.y = map_.rows+(y_origin_map_meter_ - pose.y)/map_resolution_;
}



//void PathChecker::pointToPose(const cv::Point_<int> &point, geometry_msgs::Pose2D &pose);



void PathChecker::occupancyGridToMat(const nav_msgs::OccupancyGrid &occupancy_grid)
{
    //getting info from metadata
    map_resolution_ = occupancy_grid.info.resolution;
    x_origin_map_meter_ = occupancy_grid.info.origin.position.x;
    y_origin_map_meter_ = occupancy_grid.info.origin.position.y;
    x_origin_map_pixel_ = -x_origin_map_meter_/map_resolution_;
    y_origin_map_pixel_ =  occupancy_grid.info.height+y_origin_map_meter_/map_resolution_;

    map_ = (cv::Mat_<bool>)(cv::Mat::zeros(occupancy_grid.info.width,occupancy_grid.info.height,CV_8U));
    for (size_t i=0; i<map_.rows; i++)
    {
        for (size_t j=0; j<map_.cols; j++)
        {
            size_t index = j + ((map_.cols - i - 1) * map_.rows);
            int8_t value = occupancy_grid.data[index];
            // 0 = free space --> true
            map_.at<bool>(i,j) = (value == 0);
        }
    }
    cv::imshow("Map from Service",map_);
    cv::waitKey(200);
}



//void PathChecker::poseFromTarget(geometry_msgs::Pose2D &pose);



bool PathChecker::isPathWayFree(scenario_msgs::Pose2DArray const &path2D)
{
    for(std::vector<geometry_msgs::Pose2D>::const_iterator it_pose = path2D.poses.begin();
            it_pose < path2D.poses.end()-1; it_pose++)
    {
        cv::Point_<int> pt1, pt2;
        poseToPoint(*it_pose, pt1);
        poseToPoint(*(it_pose+1), pt2);
        if(!isWayFree(pt1, pt2))
           return false;
    }
    return true;
}



bool PathChecker::isWayFree(const cv::Point_<int> &point1, const cv::Point_<int> &point2)
{
    int x_old = point1.x;
    int y_old = point1.y;

    if(!map_.at<bool>(point1))
        return false;

    for(float k = 0.1; k<=1; k+=0.1)
    {
        cv::Point_<int> pt = point1 + k*(point2 - point1);
        if(!map_.at<bool>(pt))
            return false;
    }

    return true;
}



bool PathChecker::serviceCB(path_checker::PathCheckerReq::Request &req, path_checker::PathCheckerReq::Response &res)
{
    ros::Time then = ros::Time::now();
    ROS_INFO_STREAM("Request received.");
    path_received_ = req.path_request;

    if(isPathWayFree(path_received_))
    {
        ROS_INFO_STREAM("Path is free! Process within "<<(then-ros::Time::now()).toSec()<<" sec.");
        return true;
    }

    ROS_INFO_STREAM("Unable to fit the path... Process within "<<(then-ros::Time::now()).toSec()<<" sec.");
    return false;
}



//void PathChecker::shrinkPath(const float coef, scenario_msgs::Pose2DArray &poses);
//void PathChecker::rotatePath(const float angle);




int main(int argc, char **argv)
{
    ros::init(argc, argv, "path_checker");
    ros::NodeHandle nh;

    PathChecker path_checker(nh);
    ros::ServiceServer path_checker_service = nh.advertiseService("path_checker",&PathChecker::serviceCB,&path_checker);

    ros::spinOnce();

    ROS_INFO("Ready to compute path.");


    while(ros::ok())
    {
        ros::spin();
    }


    return 0;
}
