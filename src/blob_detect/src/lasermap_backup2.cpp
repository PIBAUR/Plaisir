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
    LidarBliter()
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
      
      //laser_sub_ = n_.subscribe("/base_scan2",1,&LidarBliter::laserCb, &this);
      
      process_map();
      //cv::imshow("MAP",map_);
      cv::circle(map_, map_origin_, 5, cv::Scalar(255,0,0), 3, 8, 0);
     // cv::imshow("MAP",map_);
      cv::waitKey(20);
    }

    ~LidarBliter()
    {
    }
    
    void laserCb(const sensor_msgs::LaserScan::ConstPtr& msg);
    
    void display_laser(const sensor_msgs::LaserScan& laser)
    {
      //cv::imshow("laser1",map_laser_);
      std::cout<<"plop"<<std::endl;
      int nb = (laser.angle_max - laser.angle_min) / laser.angle_increment;
      std::cout<<nb<<std::endl;
      try
      {
        for(size_t i=0; i<= nb+1; i++)
        {
          float x = laser.ranges[i] * cos(laser.angle_min+i*laser.angle_increment);
          float y = laser.ranges[i] * sin(laser.angle_min+i*laser.angle_increment);
          x /= metadata_.resolution;
          y /= metadata_.resolution;
          cv::Point pt = cv::Point(x,y)+map_origin_;
          
          //std::cout<<" pixel = "<< (int)map_bin_.at<unsigned char>(pt) <<std::endl;
          if(map_bin_.at<unsigned char>(pt) != 0)
          {
            cv::circle(map_laser_,cv::Point(x,y)+map_origin_,3,cv::Scalar(0,0,255),-1,8,0);
          }
          
        }
      }
      catch(...)
      {
        std::cout<<"ERROR LASER"<<std::endl;
      }
      //cv::imshow("laser",map_laser_);
      cv::waitKey(100);
    }
    
    void process_map()
    {
      cv::cvtColor(map_,map_bin_,CV_BGR2GRAY);
      cv::blur(map_bin_, map_bin_, cv::Size(3,3), cv::Point(-1,-1), cv::BORDER_DEFAULT);
      cv::threshold(map_bin_,map_bin_,230,255,cv::THRESH_BINARY);
      //cv::imshow("img_process",map_bin_);
      //cv::waitKey();
    }
    
    
    void spin()
    {
      ros::spin();
    }


  ///----ATTRIBUTS----///
  public :
    ros::NodeHandle n_;
    cv::Point map_origin_;
    cv::Mat map_;
    cv::Mat map_laser_;
    cv::Mat map_bin_;
    nav_msgs::MapMetaData metadata_;
    sensor_msgs::LaserScan laser_scan_;
    ros::ServiceClient map_client_;
    nav_msgs::GetMap map_service_;
    //ros::Subscriber laser_sub_;
};
// %EndTag(CLASS_WITH_DECLARATION)%


void LidarBliter::laserCb(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  map_laser_= map_.clone();
  display_laser(*msg);
}


class myBlobDetector : cv::SimpleBlobDetector
{
  public:
    ///----METHODS----///
    
    
    myBlobDetector()
    {
      params.thresholdStep = 1;
      params.minThreshold  = 0;
      params.maxThreshold = 255;

      params.minRepeatability = 0;

      params.minDistBetweenBlobs = 1;

      params.filterByColor = false;
      //params.blobColor = 0;

      params.filterByArea = true;
      params.minArea = 1;
      params.maxArea = 100;

      params.filterByCircularity = false;
      //params.minCircularity = ;
      //params.maxCircularity = ;

      params.filterByInertia = false;
      //params.minInertiaRatio = ;
      //params.maxInertiaRatio = ;

      params.filterByConvexity = false;
      //params.minConvexity = ;
      //params.maxConvexity = ;
    }
    
    ~myBlobDetector()
    {}
    
    void mydetect(cv::Mat& mat)
    {
      cv::Mat map_blob;
      detect(mat,blobs,map_blob);
      for(cv::vector<cv::KeyPoint>::iterator blob_it = blobs.begin(); blob_it != blobs.end(); blob_it++)
      {
        cv::circle(mat,blob_it->pt,blob_it->size,cv::Scalar(0,255,0),2,8,0);
      }
      //cv::imshow("BLS",mat);
      //cv::waitKey(300);
      std::cout<<"Blob in"<<std::endl;
      cv::imshow("BLOBS",mat);
      std::cout<<"blobs out"<<std::endl;
      cv::waitKey(100);
    }
    
    ///----ATTRIBUTS----///
    cv::vector<cv::KeyPoint> blobs;
};







int main(int argc, char** argv)
{
  ros::init(argc, argv, "blob");
  myBlobDetector mbd;
  
  LidarBliter lb;
  
  ros::NodeHandle n;
  
  ros::Subscriber sub = n.subscribe("base_scan",1, &LidarBliter::laserCb, &lb);  
  
  ros::Rate loop_rate(5);
  loop_rate.sleep();
  loop_rate.sleep();
  while(ros::ok())
  {
    mbd.mydetect(lb.map_laser_);
    ros::spinOnce();
    loop_rate.sleep();
  }
  ros::spin();
  return 0;
}
