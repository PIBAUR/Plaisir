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

#define THRESH_IN_METER 0.10
#define GROUP_SIZE_MIN 2
#define GROUP_SIZE_MAX 100
#define RADIUS_DECREASE_VALUE 0.05
#define RADIUS_LASER_DISPLAY 3



// %Tag(CLASS_WITH_DECLARATION)%
class LidarBliter
{
    ///----METHODS----///
public :
    LidarBliter();
    ~LidarBliter(){};

    void laserCb(const sensor_msgs::LaserScan::ConstPtr& msg);
    void laser_filter(const sensor_msgs::LaserScan& laser);
    void group_laser_point();
    void group_filter();
    uint32_t set_id_closest(scenario_msgs::Obstacle &obs);

    void display_laser();
    void display_laser_raw(const sensor_msgs::LaserScan& laser);
    void display_group();

    void process_map();

    void spin();

    ///----ATTRIBUTS----///
public :
    ros::NodeHandle nh_;
    tf::TransformListener tf_listener_;

    cv::Point_<float> laser_origin_;
    float laser_yaw_;
    sensor_msgs::LaserScan laser_scan_;
    cv::vector< cv::Point_<float> > laser_point_;

    cv::Mat map_;
    cv::Mat map_laser_;
    cv::Mat map_group_;
    cv::Mat map_bin_;


    ros::ServiceClient map_client_;
    nav_msgs::MapMetaData metadata_;
    nav_msgs::GetMap map_service_;

    cv::vector< cv::vector< cv::Point_<float>  > > grouped_point_;

    ros::Publisher obstacles_pub_;
    scenario_msgs::ObstacleArray obstacles_;
    scenario_msgs::ObstacleArray obstacles_old_;
    uint32_t id_;
};
// %EndTag(CLASS_WITH_DECLARATION)%



LidarBliter::LidarBliter():
	        id_(1)
{
    map_ = cv::imread("/home/serveur/catkin_ws/maps/last_map.pgm",CV_LOAD_IMAGE_COLOR);
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
    try
    {
        for(size_t i=0; i<= nb+1; i++)
        {
            if (laser.ranges[i] < laser.range_max && laser.ranges[i] > laser.range_min)
            {
                float x = laser.ranges[i] * cos(laser.angle_min+i*laser.angle_increment + laser_yaw_);
                float y = laser.ranges[i] * sin(laser.angle_min+i*laser.angle_increment + laser_yaw_);
                cv::Point_<float> pt = (cv::Point_<float>(x,-y)+laser_origin_);
                pt *= 1.0/metadata_.resolution;
                //check if laser point is in free space
                if(map_bin_.at<unsigned char>(pt) != 0)
                {
                    laser_point_.push_back(cv::Point_<float>(x,-y)+laser_origin_);
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
    for(cv::vector< cv::Point_<float> >::iterator it_point = laser_point_.begin();
            it_point != laser_point_.end(); it_point += 1)
    {
        cv::Point_<float> pt(*it_point);
        pt *= 1.0/metadata_.resolution;
        cv::circle(map_laser_,pt,RADIUS_LASER_DISPLAY,cv::Scalar(0,0,255),-3,4,0);
    }
    cv::imshow("laser_filter",map_laser_);
    cv::waitKey(10);
}



void LidarBliter::display_laser_raw(const sensor_msgs::LaserScan& laser)
{
    map_laser_ = map_.clone();
    int nb = (laser.angle_max - laser.angle_min) / laser.angle_increment;
/*
    for(size_t i=0; i<= nb; i++)
    {
        if (laser.ranges[i] < laser.range_max)
        {
            float x = laser.ranges[i] * cos(laser.angle_min+i*laser.angle_increment + laser_yaw_);
            float y = laser.ranges[i] * sin(laser.angle_min+i*laser.angle_increment + laser_yaw_);
            cv::Point_<float> pt = (cv::Point_<float>(x,-y)+laser_origin_);
            pt *= 1.0/metadata_.resolution;
            cv::circle(map_laser_,pt,RADIUS_LASER_DISPLAY,cv::Scalar(0,0,255),-3,4,0);
        }
    }*/
    cv::circle(map_laser_, laser_origin_*(1.0/metadata_.resolution), RADIUS_LASER_DISPLAY, cv::Scalar(0,255,0), 3, 8, 0);
    cv::imshow("laser_raw", map_laser_);
    cv::waitKey(10);
}



void LidarBliter::process_map()
{
    cv::cvtColor(map_,map_bin_,CV_BGR2GRAY);
    cv::blur(map_bin_, map_bin_, cv::Size(3,3), cv::Point(-1,-1), cv::BORDER_DEFAULT);
    cv::threshold(map_bin_,map_bin_,230,255,cv::THRESH_BINARY);
}



void LidarBliter::group_laser_point()
{
    grouped_point_.clear();
    size_t nb_group=0;
    grouped_point_.push_back(cv::vector< cv::Point_<float> >() );

    cv::vector< cv::Point_<float> > *group = &(grouped_point_.back());

    (*group).push_back( *(laser_point_.begin()) );
    for(cv::vector< cv::Point_<float> >::iterator it_point = laser_point_.begin()+1; it_point != laser_point_.end(); it_point += 1)
    {
        float d = cv::norm(*it_point - (*group).back());

        if(d < THRESH_IN_METER)
        {
            (*group).push_back( *(it_point) );
        }
        else
        {
            grouped_point_.push_back(cv::vector< cv::Point_<float> > ());
            group = &grouped_point_.back();
            (*group).push_back( *(it_point) );
        }
    }
}



void LidarBliter::group_filter()
{
    obstacles_.header.stamp = ros::Time(0);
    //obstacles_.obstacles.clear();

    //decreasing radius for filtering and purge obstacles too old
    ROS_INFO_STREAM("Nomber of obstacles before process : "<<obstacles_.obstacles.size());

    for(std::vector< scenario_msgs::Obstacle >::iterator it_obs = obstacles_.obstacles.begin();
			it_obs != obstacles_.obstacles.end(); it_obs += 1)
	{
		it_obs->radius-=RADIUS_DECREASE_VALUE;

    	if(it_obs->radius < 0)
		{
    	    it_obs -= 1;
			obstacles_.obstacles.erase(it_obs+1);
		}
	}

    ROS_INFO_STREAM("Nomber of obstacles after decrease radius : "<<obstacles_.obstacles.size());

    // filtering group and add them to obstacles list
    obstacles_old_ = obstacles_;

    for(cv::vector< cv::vector<cv::Point_<float> > >::iterator it_group = grouped_point_.begin();
            it_group != grouped_point_.end(); it_group += 1)
    {
        if( it_group->size()>GROUP_SIZE_MIN && it_group->size()<GROUP_SIZE_MAX )
        {
            float x = it_group->begin()->x - (it_group->end()-1)->x;
            float y = it_group->begin()->y - (it_group->end()-1)->y;
            float radius = sqrt( x*x + y*y ) / 2.0;
            ROS_INFO_STREAM("radius for size="<<it_group->size()<< " : "<< radius);
            cv::Point_<float> pt_centre = cv::Point_<float>(0.0,0.0);

            for(cv::vector<cv::Point_<float> >::iterator it_point = it_group->begin(); it_point != it_group->end(); it_point += 1)
                pt_centre += *it_point;
            pt_centre *= 1.0/it_group->size();

            scenario_msgs::Obstacle obs;
            obs.x = pt_centre.x;
            obs.y = pt_centre.y;
            obs.radius = radius;

            if(obstacles_old_.obstacles.size()!=0)
            {
                set_id_closest(obs);
            }
            else
            {
				obs.id = id_++;
				obstacles_.obstacles.push_back(obs);
            }
        }
    }
    obstacles_pub_.publish(obstacles_);
}


void LidarBliter::display_group()
{
    map_group_ = map_.clone();

    for(std::vector< scenario_msgs::Obstacle >::iterator it_obs = obstacles_.obstacles.begin();
        it_obs != obstacles_.obstacles.end(); it_obs += 1)
    {
        int b = (it_obs->id * 20) % 255;
        int g = (it_obs->id * 40) % 255;
        int r = (it_obs->id * 60) % 255;
        cv::Point_<float> pt(it_obs->x, it_obs->y);
        pt *= 1.0/metadata_.resolution;
        //pt.y*=-1;
        cv::circle(map_group_, pt, it_obs->radius/metadata_.resolution, cv::Scalar(b,g,r), -3, 4, 0);
    }
    cv::imshow("group",map_group_);
    cv::waitKey(10);
}



uint32_t LidarBliter::set_id_closest(scenario_msgs::Obstacle &obs)
{
    float min_d = 100.0;
    uint32_t min_id = 0;

    for(std::vector< scenario_msgs::Obstacle >::iterator it_obs_old = obstacles_old_.obstacles.begin();
            it_obs_old != obstacles_old_.obstacles.end(); it_obs_old += 1)
    {

        float d = cv::norm( cv::Point_<float>(obs.x,obs.y) - cv::Point_<float>(it_obs_old->x,it_obs_old->y) );
        if(d < min_d)
        {

            bool already_use= false;
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
                min_d = d;
                min_id = it_obs_old->id;

            }
        }
    }

    if(min_id != 0)
    {
		if(obs.radius > obstacles_old_.obstacles[min_id].radius)
    	{
			obstacles_old_.obstacles[min_id].radius = obs.radius;
			obstacles_.obstacles.push_back(obstacles_old_.obstacles[min_id]);
    	}

    }
    else
    {
        obs.id = id_++;
        obstacles_.obstacles.push_back(obs);
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
    ROS_INFO_STREAM("tf");
    try
    {
        tf_listener_.lookupTransform( "/map", msg->header.frame_id, ros::Time(0), tf_laser);

        int xlaser = (-metadata_.origin.position.x + tf_laser.getOrigin().x());
        int ylaser = (-metadata_.origin.position.y - tf_laser.getOrigin().y());
        laser_yaw_ = tf::getYaw(tf_laser.getRotation());
        laser_origin_ = cv::Point_<float>(xlaser,ylaser);

    }
    catch (tf::TransformException ex)
    {
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
        return;
    }
    ROS_INFO_STREAM("display");
    display_laser_raw(*msg);
    /*ROS_INFO_STREAM("filter laser");
    laser_filter(*msg);
    ROS_INFO_STREAM("display laser");
    display_laser();
    ROS_INFO_STREAM("group");
    group_laser_point();
    ROS_INFO_STREAM("group filtre");
    group_filter();
    ROS_INFO_STREAM("display");
    display_group();
    */
    ROS_INFO_STREAM("done");
}



int main(int argc, char** argv)
{
    ros::init(argc, argv, "blob");

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
