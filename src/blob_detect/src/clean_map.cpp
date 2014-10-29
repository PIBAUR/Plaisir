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


class MapCleaner
{
  ///----METHODS----///
  public :
    MapCleaner(const char* file_name)
   {
      map_ = cv::imread(file_name,CV_LOAD_IMAGE_GRAYSCALE);

      
      cv::imshow("MAP",map_);
      std::cout<<"Size : "<<map_.size()<<std::endl;
      cv::waitKey();
      process_map();
      process_map2();
    }

    ~MapCleaner()
    {
    }

    void process_map()
    {
      //cv::cvtColor(map_,map_bin_,CV_BGR2GRAY);
      cv::blur(map_, map_bin_, cv::Size(5,5), cv::Point(-1,-1), cv::BORDER_DEFAULT);
      //cv::blur(map_bin_, map_bin_, cv::Size(5,5), cv::Point(-1,-1), cv::BORDER_DEFAULT);
      
      cv::Mat map_b1;
      cv::Mat map_b2;
      cv::threshold(map_bin_,map_b1,150,205,cv::THRESH_BINARY);
      cv::threshold(map_bin_,map_b2,210,255,cv::THRESH_BINARY);
      
      cv::Mat map_b = max(map_b1,map_b2);
      //cv::imshow("img_process",map_b);
      //cv::waitKey();
      
      
      cv::Mat element = cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(3,3),cv::Point(-1,-1));
      cv::Mat map_dil;
      cv::dilate(map_b,map_dil, element, cv::Point(-1,-1),1,cv::BORDER_DEFAULT);
      //cv::imshow("img_dil",map_dil);
      //cv::waitKey();
      //cv::erode(map)
      
      
      
      
      cv::Mat map_clean = max(map_,map_dil);
      cv::imshow("map_clean1",map_clean);
      cv::waitKey();
      
      
      //std::vector<int> compression_params;
      //compression_params.push_back(cv::PGM_BINARY);
      //compression_params.push_back(1);
      try 
      { 
        //imwrite("clean_map.pgm", map_clean, compression_params);
        cv::imwrite("clean_map.pgm", map_clean);
      } 
      catch ( const std::exception & e ) 
      { 
          std::cerr << e.what(); 
      }
    }
    
    void process_map2()
    {
      ////cv::cvtColor(map_,map_bin_,CV_BGR2GRAY);
      ////cv::blur(map_, map_bin_, cv::Size(5,5), cv::Point(-1,-1), cv::BORDER_DEFAULT);
      ////cv::blur(map_bin_, map_bin_, cv::Size(5,5), cv::Point(-1,-1), cv::BORDER_DEFAULT);
      //
      //cv::Mat map_b1;
      //cv::Mat map_b2;
      //cv::threshold(map_bin_,map_b1,150,205,cv::THRESH_BINARY);
      //cv::threshold(map_bin_,map_b2,210,255,cv::THRESH_BINARY);
      //
      //cv::Mat map_b = max(map_b1,map_b2);
      //cv::imshow("img_process",map_b);
      //cv::waitKey();
      //
      //
      
      cv::threshold(map_,map_bin_,210,255,cv::THRESH_BINARY);
      cv::Mat cross = cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(3,3),cv::Point(-1,-1));
      cv::Mat rect = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3,3),cv::Point(-1,-1));
      
      cv::Mat map_dil;
      cv::erode(map_bin_,map_dil, cross, cv::Point(-1,-1),1,cv::BORDER_DEFAULT);
      cv::imshow("img_ero",map_dil);
      
      cv::waitKey();
      cv::dilate(map_dil,map_dil, rect, cv::Point(-1,-1),1,cv::BORDER_DEFAULT);
      cv::imshow("img_dil",map_dil);
      cv::waitKey();
      //cv::erode(map)
      
      
      
      
      cv::Mat map_clean = max(map_,map_dil);
      cv::imshow("map_clean",map_clean);
      cv::waitKey();
    }


  ///----ATTRIBUTS----///
  public :
    cv::Mat map_;
    cv::Mat map_bin_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "blob");
  std::cout<<"Nb args :"<< argc << std::endl;
  for(size_t i = 0; i<argc;i++)
    std::cout<<"ARG #"<<i<<" : "<< argv[i] << std::endl;
  if(argc=2)
  {
    
    const char* image_file = argv[1];
    std::cout<<"Receive :"<< image_file << std::endl;
    MapCleaner mc(image_file);
  }
  else
  {
    std::cout<<"Wrong usage."<<std::endl;
    std::cout<<"Please use : rosrun blob_detect clean_map </path/to/file.pgm>"<<std::endl;
  }

  return 0;
}

