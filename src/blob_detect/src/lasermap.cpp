#include <ros/ros.h>

#include <tf/transform_listener.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/GetMap.h>
#include <nav_msgs/MapMetaData.h>
#include <geometry_msgs/TransformStamped.h>

#include <scenario_msgs/Obstacle.h>
#include <scenario_msgs/ObstacleArray.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>

#include <cstdlib>
#include <sstream>

#define THRESH_IN_METER 0.5
#define GROUP_SIZE_MIN 2
#define GROUP_SIZE_MAX 100
#define RADIUS_DECREASE_VALUE 0.05
#define PIXEL_ERODE_SIZE 3
#define MATCH_DIST_MAX 0.50

//
// %Tag(CLASS_WITH_DECLARATION)%
class LidarBliter
{
    ///----METHODS----///
public :
    LidarBliter();
    ~LidarBliter(){};

    void laserCb(const sensor_msgs::LaserScan::ConstPtr& msg);
    void laser_filter(const sensor_msgs::LaserScan& laser);
    void display_laser();
    void display_laser_raw(const sensor_msgs::LaserScan& laser);
    void group_laser_point();
    void display_group();
    void process_map();
    void set_id_closest(scenario_msgs::Obstacle &obs);
    void set_id_closest();
    void spin();

    ///----ATTRIBUTS----///
public :
    ros::NodeHandle nh_;
    tf::TransformListener tf_listener_;

    cv::Point laser_origin_;
    float laser_yaw_;
    sensor_msgs::LaserScan laser_scan_;
    cv::vector<cv::Point> laser_point_;

    cv::Mat map_;
    cv::Mat map_laser_;
    //cv::Mat map_group_;
    cv::Mat map_bin_;


    ros::ServiceClient map_client_;
    nav_msgs::MapMetaData metadata_;
    nav_msgs::GetMap map_service_;

    cv::vector< cv::vector<cv::Point> > grouped_point_;

    ros::Publisher obstacles_pub_;
    scenario_msgs::ObstacleArray obstacles_;
    scenario_msgs::ObstacleArray obstacles_old_;
    scenario_msgs::ObstacleArray obstacles_old_bis_;
    uint32_t id_;
};
// %EndTag(CLASS_WITH_DECLARATION)%


LidarBliter::LidarBliter():
		id_(1)
{
    std::stringstream ss;
    ss << std::getenv("HOME") <<"/catkin_ws/maps/last_map.pgm";
    map_ = cv::imread(ss.str(),CV_LOAD_IMAGE_COLOR);
    map_laser_ = map_.clone();
    obstacles_.obstacles.clear();
    obstacles_old_.obstacles.clear();
    obstacles_.header.frame_id = "/map";
    obstacles_pub_ = nh_.advertise<scenario_msgs::ObstacleArray>("obstacles",1);
    ros::ServiceClient map_client = nh_.serviceClient<nav_msgs::GetMap>("/static_map");
    nav_msgs::GetMap map_service;
    if(map_client.call(map_service))
    {
        metadata_ = map_service.response.map.info;
    }
    else
    {
        ROS_ERROR("Failed to call service map");
    }
    process_map();
}

void LidarBliter::laser_filter(const sensor_msgs::LaserScan& laser)
{
    laser_point_.clear();
    int nb = (laser.angle_max - laser.angle_min) / laser.angle_increment;
    ROS_INFO_STREAM("Nombre de point dans le scan : "<<nb);
    try
    {

        for(size_t i=0; i<= nb+1; i++)
        {
            if (laser.ranges[i] < laser.range_max && laser.ranges[i] > laser.range_min)
            {
                float x = laser.ranges[i] * cos(laser.angle_min+i*laser.angle_increment + laser_yaw_);
                float y = laser.ranges[i] * sin(laser.angle_min+i*laser.angle_increment + laser_yaw_);
                x /= metadata_.resolution;
                y /= -metadata_.resolution;
                cv::Point pt = cv::Point(x,y)+laser_origin_;
                if(map_bin_.at<unsigned char>(pt) != 0)
                {
                    laser_point_.push_back(cv::Point(x,y)+laser_origin_);
                }
            }
        }
    }
    catch(...)
    {
    }
}

void LidarBliter::display_laser()
{
    map_laser_ = map_.clone();
    for(cv::vector<cv::Point>::iterator it = laser_point_.begin(); it != laser_point_.end(); it += 1)
    {
        cv::circle(map_laser_,*it,3,cv::Scalar(0,0,255),-3,4,0);
    }
    cv::imshow("laser_filter",map_laser_);
    cv::waitKey(10);
}



void LidarBliter::display_laser_raw(const sensor_msgs::LaserScan& laser)
{
    map_laser_ = map_.clone();

    int nb = (laser.angle_max - laser.angle_min) / laser.angle_increment;
    std::cout<<nb<<std::endl;
    try
    {
        for(size_t i=0; i<= nb; i++)
        {
            if (laser.ranges[i] < laser.range_max && laser.ranges[i] < laser.range_min)
            {
                float x = laser.ranges[i] * cos(laser.angle_min+i*laser.angle_increment + laser_yaw_);
                float y = laser.ranges[i] * sin(laser.angle_min+i*laser.angle_increment + laser_yaw_);
                x /= metadata_.resolution;
                y /= -metadata_.resolution;
                cv::Point pt = cv::Point(x,y)+laser_origin_;
                cv::circle(map_laser_,pt,3,cv::Scalar(0,0,255),-3,4,0);
            }
        }
    }
    catch(...)
    {
    }
    cv::circle(map_laser_, laser_origin_, 2, cv::Scalar(0,255,0), 3, 8, 0);
    cv::imshow("laser_raw",map_laser_);
    cv::waitKey(10);
}


void LidarBliter::process_map()
{
    cv::cvtColor(map_,map_bin_,CV_BGR2GRAY);
    cv::blur(map_bin_, map_bin_, cv::Size(3,3), cv::Point(-1,-1), cv::BORDER_DEFAULT);
    cv::threshold(map_bin_,map_bin_,240,255,cv::THRESH_BINARY);


    cv::Mat element = getStructuringElement( cv::MORPH_ELLIPSE,
                                         cv::Size( 2*PIXEL_ERODE_SIZE + 1, 2*PIXEL_ERODE_SIZE + 1 ),
                                         cv::Point( 0, 0 ) );

    cv::erode(map_bin_,map_bin_,element);
    //cv::erode(map_bin_, map_bin_, element,cv::Point(-1,-1), 1, cv::BORDER_CONSTANT, cv::morphologyDefaultBorderValue() );
    //cv::imshow("map_process",map_bin_);
}

void LidarBliter::group_laser_point()
{
    grouped_point_.clear();
    size_t nb_group=0;
    grouped_point_.push_back(cv::vector<cv::Point>());
    cv::vector<cv::Point> *group = &(grouped_point_.back());
    (*group).push_back( *(laser_point_.begin()) );
    for(cv::vector<cv::Point>::iterator it_point = laser_point_.begin()+1; it_point != laser_point_.end(); it_point += 1)
    {
        float d = cv::norm(*it_point - (*group).back());

        if(d < (THRESH_IN_METER/metadata_.resolution))
        {
            (*group).push_back( *(it_point) );
        }
        else
        {
            grouped_point_.push_back(cv::vector<cv::Point>());
            group = &grouped_point_.back();
            (*group).push_back( *(it_point) );
        }
    }
}

void LidarBliter::display_group()
{
    ////////////////////////////////////////////////////
    //Cleaned for detection use only. without tracking//
    ////////////////////////////////////////////////////

    //map_group_ = map_.clone();
        obstacles_old_bis_.obstacles.clear();
        obstacles_old_bis_.obstacles = obstacles_old_.obstacles;

        obstacles_old_.obstacles.clear();
        obstacles_old_.obstacles = obstacles_.obstacles;

        obstacles_.obstacles.clear();
        obstacles_.header.stamp = ros::Time(0);

    //id_ = 1;
    /*
    for(std::vector< scenario_msgs::Obstacle >::iterator it_obs = obstacles_.obstacles.begin();
			it_obs != obstacles_.obstacles.end(); it_obs += 1)
	{
		it_obs->radius-=RADIUS_DECREASE_VALUE;

    	if(it_obs->radius < 0)
		{
			obstacles_.obstacles.erase(it_obs);
		}
	}

    obstacles_old_ = obstacles_;
    */


    ROS_INFO_STREAM(" nb_group of group : "<<grouped_point_. size());
    for(cv::vector< cv::vector<cv::Point> >::iterator it_group = grouped_point_.begin();
        it_group != grouped_point_.end(); it_group += 1)
    {
        std::cout<<" size of group : "<<it_group->size()<<std::endl;
        if( it_group->size()>=GROUP_SIZE_MIN
            && it_group->size()<=GROUP_SIZE_MAX )
        {
            float radius = cv::norm(it_group->begin() - it_group->end()) / 2.0;

            cv::Point pt_centre = cv::Point(0,0);
            for(cv::vector<cv::Point>::iterator it_point = it_group->begin();
                    it_point != it_group->end(); it_point += 1)
            {
                pt_centre += *it_point;
            }
            pt_centre *= 1.0/it_group->size();

            scenario_msgs::Obstacle obs;
            obs.x = pt_centre.x*metadata_.resolution+metadata_.origin.position.x;
            obs.y = -pt_centre.y*metadata_.resolution-metadata_.origin.position.y;;
            obs.radius = radius*metadata_.resolution;


            if(obstacles_old_.obstacles.size()!=0)
            {
                obs.id = 0;
                set_id_closest(obs);
            }
            else
            {
                obs.id = id_++;
            }
            obstacles_.obstacles.push_back(obs);
            /*
            int b = 0;
            int g = (obs.id * 40) % 255;
            int r = (obs.id * 40) % 255;
            cv::circle(map_group_,pt_centre,radius,cv::Scalar(b,g,r),-3,4,0);
           */
        }
    }
    /*if(obstacles_old_.obstacles.size()!=0)
    {
        set_id_closest();
    }
    */


    obstacles_pub_.publish(obstacles_);
    //cv::imshow("group",map_group_);
    //cv::waitKey(10);
    id_ = id_%1000;
}



void LidarBliter::set_id_closest(scenario_msgs::Obstacle &obs)
{
    float min_d = MATCH_DIST_MAX/metadata_.resolution;
    uint32_t min_id = 0;

    for(std::vector< scenario_msgs::Obstacle >::iterator it_obs_old = obstacles_old_.obstacles.begin();
            it_obs_old != obstacles_old_.obstacles.end(); it_obs_old += 1)
    {

        float d = cv::norm( cv::Point(obs.x,obs.y) - cv::Point(it_obs_old->x,it_obs_old->y) );
        if(d < min_d)
        {
            bool already_use = false;
            for(std::vector< scenario_msgs::Obstacle >::iterator it_obs = obstacles_.obstacles.begin();
                        it_obs != obstacles_.obstacles.end(); it_obs += 1)
            {
                if(it_obs->id == it_obs_old->id)
                {
                    already_use = true;
                    break;
                }
            }
            if(!already_use)
            {
            	//ROS_INFO_STREAM();
                min_d = d;
                min_id = it_obs_old->id;

            }
        }
    }

    for(std::vector< scenario_msgs::Obstacle >::iterator it_obs_old = obstacles_old_bis_.obstacles.begin();
                it_obs_old != obstacles_old_bis_.obstacles.end(); it_obs_old += 1)
    {

        float d = cv::norm( cv::Point(obs.x,obs.y) - cv::Point(it_obs_old->x,it_obs_old->y) );
        if(d < min_d)
        {
            bool already_use = false;
            for(std::vector< scenario_msgs::Obstacle >::iterator it_obs = obstacles_.obstacles.begin();
                        it_obs != obstacles_.obstacles.end(); it_obs += 1)
            {
                if(it_obs->id == it_obs_old->id)
                {
                    already_use = true;
                    break;
                }
            }
            if(!already_use)
            {
                //ROS_INFO_STREAM();
                min_d = d;
                min_id = it_obs_old->id;

            }
        }
    }

    if(min_id != 0)
    {
        obs.id = min_id;
    }
    else
    {
        obs.id = id_++;
    }
}




void LidarBliter::set_id_closest()
{

    if(obstacles_old_.obstacles.size() != 0)
    {
        for(std::vector< scenario_msgs::Obstacle >::iterator it_obs_old = obstacles_old_.obstacles.begin();
            it_obs_old != obstacles_old_.obstacles.end(); it_obs_old += 1)
        {
            float min_d = MATCH_DIST_MAX/metadata_.resolution;
            uint32_t min_id = 0;
            std::vector< scenario_msgs::Obstacle >::iterator nearest_obs;

            for(std::vector< scenario_msgs::Obstacle >::iterator it_obs = obstacles_.obstacles.begin();
                it_obs != obstacles_.obstacles.end(); it_obs += 1)
            {
                float d = cv::norm( cv::Point(it_obs->x,it_obs->y) - cv::Point(it_obs_old->x,it_obs_old->y) );

                if(d < min_d && it_obs->id!=0)
                {
                    min_d = d;
                    nearest_obs = it_obs;
                }
            }
            nearest_obs->id = it_obs_old->id;
        }
    }

    if(obstacles_old_bis_.obstacles.size() != 0)
    {
        for(std::vector< scenario_msgs::Obstacle >::iterator it_obs_old_bis = obstacles_old_bis_.obstacles.begin();
            it_obs_old_bis != obstacles_old_bis_.obstacles.end(); it_obs_old_bis += 1)
        {
            float min_d = MATCH_DIST_MAX/metadata_.resolution;
            uint32_t min_id = 0;
            std::vector< scenario_msgs::Obstacle >::iterator nearest_obs;

            for(std::vector< scenario_msgs::Obstacle >::iterator it_obs = obstacles_.obstacles.begin();
                    it_obs != obstacles_.obstacles.end(); it_obs += 1)
            {
                float d = cv::norm( cv::Point(it_obs->x,it_obs->y) - cv::Point(it_obs_old_bis->x,it_obs_old_bis->y) );

                if(d < min_d && it_obs->id!=0)
                {
                    min_d = d;
                    nearest_obs = it_obs;
                }
            }
            nearest_obs->id = it_obs_old_bis->id;
        }
    }
    if(obstacles_.obstacles.size() != 0)
    {
        for(std::vector< scenario_msgs::Obstacle >::iterator it_obs = obstacles_.obstacles.begin();
            it_obs != obstacles_.obstacles.end(); it_obs += 1)
        {

            if(it_obs->id==0)
            {
                it_obs->id = id_++;
            }
        }
    }
}




void LidarBliter::spin()
{
    ros::spin();
}



void LidarBliter::laserCb(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    map_laser_= map_.clone();
    tf::StampedTransform tf_laser;

    try
    {
        tf_listener_.lookupTransform("/map", msg->header.frame_id, ros::Time(0), tf_laser);
        int xlaser = (-metadata_.origin.position.x + tf_laser.getOrigin().x())/metadata_.resolution;
        int ylaser = (-metadata_.origin.position.y -tf_laser.getOrigin().y())/metadata_.resolution;
        laser_yaw_ = tf::getYaw(tf_laser.getRotation());
        laser_origin_ = cv::Point(xlaser,ylaser);

    }
    catch (tf::TransformException ex)
    {
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
        return;
    }
    //display_laser_raw(*msg);
    laser_filter(*msg);
    //display_laser();
    group_laser_point();
    display_group();
}





int main(int argc, char** argv)
{
    ros::init(argc, argv, "lasermap");

    LidarBliter lb;

    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("scan",1, &LidarBliter::laserCb, &lb);

    ros::Rate loop_rate(5);
    loop_rate.sleep();
    loop_rate.sleep();
    while(ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
    ros::spin();
    return 0;
}
