#include <ros/ros.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/GetMap.h>
#include <nav_msgs/MapMetaData.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>



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
    void group_laser_point();
    void display_group();
    void process_map();
    void spin();

    ///----ATTRIBUTS----///
public :
    ros::NodeHandle n_;
    cv::Point map_origin_;
    cv::Mat map_;
    cv::Mat map_laser_;
    cv::Mat map_group_;
    cv::Mat map_bin_;
    nav_msgs::MapMetaData metadata_;
    sensor_msgs::LaserScan laser_scan_;
    ros::ServiceClient map_client_;
    nav_msgs::GetMap map_service_;
    cv::vector<cv::Point> laser_point_;
    cv::vector< cv::vector<cv::Point> > grouped_point_;
    //ros::Subscriber laser_sub_;
};
// %EndTag(CLASS_WITH_DECLARATION)%


LidarBliter::LidarBliter()
{
    map_ = cv::imread("/home/serveur/catkin_ws/maps/map.pgm",CV_LOAD_IMAGE_COLOR);
    map_laser_ = map_.clone();
    ros::ServiceClient map_client = n_.serviceClient<nav_msgs::GetMap>("/static_map");
    nav_msgs::GetMap map_service;
    if(map_client.call(map_service))
    {
        metadata_ = map_service.response.map.info;
    }
    else
    {
        ROS_ERROR("Failed to call service map");
    }
    int xori = -metadata_.origin.position.x/metadata_.resolution;
    int yori = -metadata_.origin.position.y/metadata_.resolution;
    map_origin_ = cv::Point(xori,yori);
    
    process_map();
    //cv::imshow("MAP",map_);
    cv::circle(map_, map_origin_, 5, cv::Scalar(255,0,0), 3, 8, 0);
    // cv::imshow("MAP",map_);
    cv::waitKey(20);
}

void LidarBliter::laser_filter(const sensor_msgs::LaserScan& laser)
{
    laser_point_.clear();
    int nb = (laser.angle_max - laser.angle_min) / laser.angle_increment;
    std::cout<<nb<<std::endl;
    try
    {
        for(size_t i=0; i<= nb+1; i++)
        {
            if (laser.ranges[i] < laser.range_max && laser.ranges[i] > laser.range_min)
            {
                float x = laser.ranges[i] * cos(laser.angle_min+i*laser.angle_increment);
                float y = laser.ranges[i] * sin(laser.angle_min+i*laser.angle_increment);
                x /= metadata_.resolution;
                y /= metadata_.resolution;
                cv::Point pt = cv::Point(x,y)+map_origin_;
                if(map_bin_.at<unsigned char>(pt) != 0)
                {
                    laser_point_.push_back(cv::Point(x,y)+map_origin_);
                }
            }
        }
    }
    catch(...)
    {
    }
    //cv::imshow("laser",map_laser_);
    //cv::waitKey(10);
}
    
void LidarBliter::display_laser()
{
    map_laser_ = map_.clone();
    for(cv::vector<cv::Point>::iterator it = laser_point_.begin(); it != laser_point_.end(); it += 1)
    {
        cv::circle(map_laser_,*it,3,cv::Scalar(0,0,255),-3,4,0);
    }
    cv::imshow("laser",map_laser_);
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
    std::cout<< "plop group laser point IN" <<std::endl;
    grouped_point_.clear();
    size_t nb_group=0;
    grouped_point_.push_back(cv::vector<cv::Point>());
    cv::vector<cv::Point> *group = &(grouped_point_.back());
    (*group).push_back( *(laser_point_.begin()) );
    std::cout<< "plop group laser point  1 " <<std::endl;
    for(cv::vector<cv::Point>::iterator it = laser_point_.begin()+1; it != laser_point_.end(); it += 1)
    {
        std::cout<<"test it : "<<it->x<<"\t"<<it->y<<std::endl;
        std::cout<<"test it_grouped : "<<group->back().x<<"\t"<<group->back().y<<std::endl;
        float d =  sqrt( (float)((it->x - (*group).back().x)*(it->x - (*group).back().x)
        + (it->y - (*group).back().y)*(it->y - (*group).back().y) ));
        std::cout<< "plop group laser point float" <<std::endl;
        if(d < (0.30/metadata_.resolution))
        {
            (*group).push_back( *(it) );
            std::cout<< "plop group laser point IF" <<std::endl;
        }
        else
        {
            std::cout<< "plop group laser point else in" <<std::endl;
            grouped_point_.push_back(cv::vector<cv::Point>());
            std::cout<< "plop group laser point else 1" <<std::endl;
            group = &grouped_point_.back();
            std::cout<< "plop group laser point else 2" <<std::endl;
            (*group).push_back( *(it) );
            std::cout<< "plop group laser point else out" <<std::endl;
        }
    }
    std::cout<< "plop group laser point OUT" <<std::endl;
}
    
void LidarBliter::display_group()
{
    std::cout<< "display group laser point IN" <<std::endl;
    map_group_ = map_.clone();
    std::cout<<" nb_group of group : "<<grouped_point_. size()<<std::endl;
    for(cv::vector< cv::vector<cv::Point> >::iterator it_group = grouped_point_.begin();
    it_group != grouped_point_.end(); it_group += 1)
    {
        std::cout<<" size of group : "<<it_group->size()<<std::endl;
        if(it_group->size()>5 && it_group->size()<20)
        {
            int b = rand() % 255;
            int g = rand() % 255;
            int r = rand() % 255;
            cv::Point pt_centre = cv::Point(0,0);
            std::cout<<"bgr = "<<b<<"\t"<<g<<"\t"<<r<<"\t"<<std::endl;
            for(cv::vector<cv::Point>::iterator it_point = it_group->begin();
            it_point != it_group->end(); it_point += 1)
            {
                pt_centre += *it_point;
            }
            pt_centre *= 1.0/it_group->size();
            cv::circle(map_group_,pt_centre,5,cv::Scalar(b,g,r),-3,4,0);
        }
    }
    cv::imshow("group",map_group_);
    cv::waitKey(10);
    std::cout<< "display group laser point OUT" <<std::endl;
}


void LidarBliter::spin()
{
    ros::spin();
}



void LidarBliter::laserCb(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    map_laser_= map_.clone();
    laser_filter(*msg);
    display_laser();
    group_laser_point();
    display_group();
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
