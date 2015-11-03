#include "PathChecker.h"



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



void PathChecker::poseToPoint(const geometry_msgs::Pose &pose, cv::Point_<int> &point)
{
    point.x = -(x_origin_map_meter_ - pose.position.x)/map_resolution_;
    point.y = map_.rows+(y_origin_map_meter_ - pose.position.y)/map_resolution_;
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



bool PathChecker::isPathWayFree(geometry_msgs::PoseArray const &path)
{
    for(std::vector<geometry_msgs::Pose>::const_iterator it_pose = path.poses.begin();
            it_pose < path.poses.end()-1; it_pose++)
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
    path_working_ = req.path_request;
    pose_target_ = req.target_pose;

    ROS_INFO_STREAM("Request received. Path starting from : "
            << path_working_.poses.begin()->position.x << " ; " << path_working_.poses.begin()->position.x
            << " . Target : " << pose_target_.x << " ; " << pose_target_.y);

    //check if path already ok
    if(isPathWayFree(path_working_))
    {
        ROS_INFO_STREAM("Path is free! Process within "<<(then-ros::Time::now()).toSec()<<" sec.");
        res.path_result = path_working_;
        res.is_possible = true;
        return true;
    }
    // else modify path
    else
    {
        //try to shrink with rotate first
        for(float k = 1.0 - SHRINK_DEFAULT_STEP ; k >= SHRINK_DEFAULT_LIMIT ; k -= SHRINK_DEFAULT_STEP)
        {
            ROS_DEBUG_STREAM("Try shrinked to "<< k*100<<"%.");
            //reset path with path received
            path_working_ = req.path_request;
            shrinkPath(k);
            if(isPathWayFree(path_working_))
            {
                ROS_INFO_STREAM("Path shrinked to "<< k*100<<"%. Process within "
                                <<(then-ros::Time::now()).toSec()<<" sec.");
                res.path_result = path_working_;
                res.is_possible = true;
                return true;
            }
        }
        //then, try to rotate and shrink
        for(float w = ROTATE_DEFAULT_STEP ; w >= ROTATE_DEFAULT_LIMIT ; w += ROTATE_DEFAULT_STEP)
        {
            ROS_DEBUG_STREAM("Try rotate by "<< w*180.0/PI <<"°.");
            //reset path with path received
            path_working_ = req.path_request;
            // rotate and check
            rotatePath(w);
            if(isPathWayFree(path_working_))
            {
                ROS_INFO_STREAM("Path rotate by "<< w*180.0/PI <<"°. Process within "
                                <<(then-ros::Time::now()).toSec()<<" sec.");
                res.path_result = path_working_;
                res.is_possible = true;
                return true;
            }
            //try to shrink
            geometry_msgs::PoseArray rotated_path = path_working_;
            for(float k = 1.0 - SHRINK_DEFAULT_STEP ; k >= SHRINK_DEFAULT_LIMIT ; k -= SHRINK_DEFAULT_STEP)
            {
                //reset path with path after rotation
                ROS_DEBUG_STREAM("Try rotate by "<< w*180.0/PI <<"° and shrinked to "<< k*100<<"%.");
                path_working_ = rotated_path;
                shrinkPath(k);
                if(isPathWayFree(path_working_))
                {
                    ROS_INFO_STREAM("Path rotate by "<< w*180.0/PI <<"° and shrinked to "<< k*100
                                    <<"%. Process within "<<(then-ros::Time::now()).toSec()<<" sec.");
                    res.path_result = path_working_;
                    res.is_possible = true;
                    return true;
                }
            }
        }
    }

    ROS_INFO_STREAM("Unable to fit the path... Process within "<<(then-ros::Time::now()).toSec()<<" sec.");
    res.is_possible = false;
    return true;
}



void PathChecker::shrinkPath(const float coef)
{
    for(std::vector<geometry_msgs::Pose>::iterator it_pose = path_working_.poses.begin();
            it_pose < path_working_.poses.end()-1; it_pose++)
    {
        it_pose->position.x = (coef -1.0) * (-pose_target_.x) + coef * (it_pose->position.x);
        it_pose->position.y = (coef -1.0) * (-pose_target_.y) + coef * (it_pose->position.y);
    }
}



void PathChecker::rotatePath(const float angle)
{
    for(std::vector<geometry_msgs::Pose>::iterator it_pose = path_working_.poses.begin();
            it_pose < path_working_.poses.end()-1; it_pose++)
    {
        float cos_a = cos(angle);
        float sin_a = sin(angle);

        it_pose->position.x = (cos_a - sin_a - 1.0)*(-pose_target_.x) + cos_a*(it_pose->position.x) - sin_a*(it_pose->position.y);
        it_pose->position.y = (cos_a + sin_a - 1.0)*(-pose_target_.y) + sin_a*(it_pose->position.x) + cos_a*(it_pose->position.y);
    }
}




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
