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




static const std::string OPENCV_WINDOW = "Image window";

sensor_msgs::LaserScan laser;
//bool laser_set = false;
cv::Mat img_map;
int xori;
int yori;


void laserCb(const sensor_msgs::LaserScan& l){
  std::cout << "Plop CB1" << std::endl;
  laser = l;
  std::cout << "Plop CB2" << std::endl;
  cv::Mat img_laser = img_map;
  int nb = (l.angle_max - l.angle_min) / l.angle_increment;
  std::cout << "Plop 4 "<< nb << std::endl;
  try
  {
    for(size_t i=0; i< nb; i++)
    {
      std::cout << "Plop for "<< i << std::endl;
      int x = l.ranges[i] * cos(l.angle_min+i*l.angle_increment) / 0.05 + xori;
      int y = l.ranges[i] * sin(l.angle_min+i*l.angle_increment) / 0.05 + yori;
      cv::circle(img_laser,cv::Point(x,y),2,cv::Scalar(0,0,255),1,8,0);
    }
  }
  catch(...)
  {
    std::cout<<"ERROR LASER"<<std::endl;
  }
  
  
  cv::imshow("laser",img_laser);
  cv::waitKey(200);
}



int main(int argc, char** argv)
{
  ros::init(argc, argv, "blob");
  ros::NodeHandle n;
  
  
  
  ///---------------------------------------------------------------///
  ///---------------------------------------------------------------///
  ///---------------------------------------------------------------///
  ///--------------------LASER DISPLAY ON MAP-----------------------///
  ///---------------------------------------------------------------///
  ///---------------------------------------------------------------///
  ///---------------------------------------------------------------///
  
  laser.angle_min = 0;
  laser.angle_max = 0;
  laser.angle_increment = 0;
  ///-----------------Loading map image and data--------------------///
  img_map = cv::imread("/home/serveur/catkin_ws/maps/sentier2.pgm",CV_LOAD_IMAGE_COLOR);
  ros::ServiceClient map_client = n.serviceClient<nav_msgs::GetMap>("/static_map");
  nav_msgs::GetMap map_service;
  nav_msgs::MapMetaData map_meta_data;
  std::string frame = "Map ";
  if (map_client.call(map_service))
  {
    ROS_INFO("%d", (int)map_service.response.map.info.resolution);
    map_meta_data = map_service.response.map.info;
  }
  else
  {
    ROS_ERROR("Failed to call service add_two_ints");
    return -1;
  }
  ///------------------display map-------------------///
  xori = -map_meta_data.origin.position.x/map_meta_data.resolution;
  yori = -map_meta_data.origin.position.y/map_meta_data.resolution;
  cv::circle(img_map,cv::Point(xori,yori),5,cv::Scalar(255,0,0),3,8,0);
  cv::imshow(frame,img_map);
  ros::Subscriber laser_sub = n.subscribe("/base_scan2",1,laserCb);

  ///---------------------------------------------------------------///
  ///---------------------------------------------------------------///
  ///---------------------------------------------------------------///
  ///-------------------TEST DE MOI QUI MARCHE----------------------///
  ///---------------------------------------------------------------///
  ///---------------------------------------------------------------///
  ///---------------------------------------------------------------///
  /*
  cv::vector<cv::Mat*> images;
  cv::vector<std::string> frames;
  size_t cpt_frames;

  ///---------------------load images-----------------///
  images.push_back(new cv::Mat(cv::imread("/home/serveur/catkin_ws/src/blob_detect/blob_image/1.jpg",
                        CV_LOAD_IMAGE_COLOR)));
  frames.push_back("1.jpg");

  images.push_back(new cv::Mat(cv::imread("/home/serveur/catkin_ws/src/blob_detect/blob_image/2.jpg",
                        CV_LOAD_IMAGE_COLOR)));
  frames.push_back("2.jpg");

  images.push_back(new cv::Mat(cv::imread("/home/serveur/catkin_ws/src/blob_detect/blob_image/3.jpg",
                        CV_LOAD_IMAGE_COLOR)));
  frames.push_back("3.jpg");

  images.push_back(new cv::Mat(cv::imread("/home/serveur/catkin_ws/src/blob_detect/blob_image/4.png",
                        CV_LOAD_IMAGE_COLOR)));
  frames.push_back("4.pgn");

  images.push_back(new cv::Mat(cv::imread("/home/serveur/catkin_ws/src/blob_detect/blob_image/5.png",
                        CV_LOAD_IMAGE_COLOR)));
  frames.push_back("5.pgn");

  images.push_back(new cv::Mat(cv::imread("/home/serveur/catkin_ws/src/blob_detect/blob_image/6.png",
                        CV_LOAD_IMAGE_COLOR)));
  frames.push_back("6.pgn");

  images.push_back(new cv::Mat(cv::imread("/home/serveur/catkin_ws/src/blob_detect/blob_image/7.png",
                        CV_LOAD_IMAGE_COLOR)));
  frames.push_back("7.pgn");

  images.push_back(new cv::Mat(cv::imread("/home/serveur/catkin_ws/src/blob_detect/blob_image/8.pgm",
                        CV_LOAD_IMAGE_COLOR)));
  frames.push_back("8.pgm");


  ///---------------------display image-----------------///
  cpt_frames = 0;
  for(cv::vector<cv::Mat*>::iterator it_image = images.begin(); it_image != images.end(); it_image++){
    cv::imshow(frames[cpt_frames],**it_image);
    cpt_frames++;
  }
  cpt_frames = 0;

  ///-------------------Blob Detection setup--------------///
  cv::SimpleBlobDetector::Params params;
  //params.thresholdStep = 10;
  //params.minThreshold  = 0;
  //params.maxThreshold = 1;

  params.minRepeatability = 2;

  params.minDistBetweenBlobs = 5;

  params.filterByColor = true;
  params.blobColor = 0;

  params.filterByArea = true;
  params.minArea = 200;
  params.maxArea = 50000;

  params.filterByCircularity = false;
  //params.minCircularity = ;
  //params.maxCircularity = ;

  params.filterByInertia = false;
  //params.minInertiaRatio = ;
  //params.maxInertiaRatio = ;

  params.filterByConvexity = false;
  //params.minConvexity = ;
  //params.maxConvexity = ;

  cv::SimpleBlobDetector myBlobDetector(params);
  cv::vector<cv::KeyPoint> myBlobs;



  /// for each images
  for(cv::vector<cv::Mat*>::iterator it_image = images.begin(); it_image != images.end(); it_image++){
    myBlobDetector.detect(**it_image, myBlobs);

    cv::Mat blobImg;
    cv::drawKeypoints(**it_image, myBlobs, blobImg);
    for(cv::vector<cv::KeyPoint>::iterator blobIterator = myBlobs.begin(); blobIterator != myBlobs.end(); blobIterator++){
      cv::circle(blobImg,blobIterator->pt,blobIterator->size,cv::Scalar(255,0,0),2,8,0);
    }

    cv::imshow(frames[cpt_frames] + "bin",blobImg);
    cpt_frames++;
  }
  cpt_frames = 0;
  */






  ///---------------------------------------------------------------///
  ///---------------------------------------------------------------///
  ///---------------------------------------------------------------///
  ///-------------------------TEST DE MOI---------------------------///
  ///---------------------------------------------------------------///
  ///---------------------------------------------------------------///
  ///---------------------------------------------------------------///

  //  MyBlobDetector bd;
  /*
  std::cout<<"plop1"<<std::endl;
  cv::Mat img = cv::imread("/home/serveur/catkin_ws/src/blob_detect/blob_image/6.png",CV_LOAD_IMAGE_GRAYSCALE);
  //std::cout<<"plop2 "<< img.at[10][10]<<std::endl;
  std::cout<<"plop2 "<<std::endl;
  cv::imshow("blob1",img);
  std::cout<<"plop3"<<std::endl;
  cv::waitKey(100);

  cv::Mat img_bin;
  cv::Mat img_bin2 = img;
  cv::threshold(img,img_bin,250,255,cv::THRESH_BINARY_INV);

  cv::imshow("blob_bin",img_bin);
  /*
  cv::SimpleBlobDetector sbd;
  cv::vector<cv::Center> cent;
  sbd.findBlobs(img_bin2,img_bin,cent);
  */


  ///---------------------------------------------------------------///
  ///---------------------------------------------------------------///
  ///---------------------------------------------------------------///
  ///-------------------TEST ET EXEMPLES TROUVES 1------------------///
  ///---------------------------------------------------------------///
  ///---------------------------------------------------------------///
  ///---------------------------------------------------------------///

/*
  // set up the parameters (check the defaults in opencv's code in blobdetector.cpp)
cv::SimpleBlobDetector::Params params;;
params.filterByInertia = false;
params.filterByConvexity = false;
params.filterByColor = false;
params.filterByCircularity = false;
params.filterByArea = false;

// ... any other params you don't want default value

// set up and create the detector using the parameters
cv::Ptr<cv::FeatureDetector> blob_detector = new cv::SimpleBlobDetector(params);
blob_detector->create("SimpleBlob");
*/
/*
// detect!
cv::vector<cv::KeyPoint> keypoints;
blob_detector->detect(img_bin, keypoints);

// extract the x y coordinates of the keypoints:

for (int i=0; i<keypoints.size(); i++){
    float X=keypoints[i].pt.x;
    float Y=keypoints[i].pt.y;
    std::cout<< " " << X << "\t" << Y <<std::endl;
}

  cv::drawKeypoints(img_bin,keypoints,img_bin2);
  cv::imshow("blob detect",img_bin2);

 std::cout<< " " << keypoints.size() <<std::endl;

  */


  ///---------------------------------------------------------------///
  ///---------------------------------------------------------------///
  ///---------------------------------------------------------------///
  ///---------------------------STRUCT PARAMS-----------------------///
  ///---------------------------------------------------------------///
  ///---------------------------------------------------------------///
  ///---------------------------------------------------------------///
  /*
  {
    Params();
    float thresholdStep;
    float minThreshold;
    float maxThreshold;
    size_t minRepeatability;
    float minDistBetweenBlobs;

    bool filterByColor;
    uchar blobColor;

    bool filterByArea;
    float minArea, maxArea;

    bool filterByCircularity;
    float minCircularity, maxCircularity;

    bool filterByInertia;
    float minInertiaRatio, maxInertiaRatio;

    bool filterByConvexity;
    float minConvexity, maxConvexity;
};
  */


  ///---------------------------------------------------------------///
  ///---------------------------------------------------------------///
  ///---------------------------------------------------------------///
  ///-----------------------EXEMPLE QUI "MARCHE"--------------------///
  ///---------------------------------------------------------------///
  ///---------------------------------------------------------------///
  ///---------------------------------------------------------------///
  /*
  cv::SimpleBlobDetector::Params params;
  params.minDistBetweenBlobs = 10.0;  // minimum 10 pixels between blobs
  params.filterByArea = true;         // filter my blobs by area of blob
  params.minArea = 10.0;              // min 20 pixels squared
  params.maxArea = 50000.0;             // max 500 pixels squared
  cv::SimpleBlobDetector myBlobDetector(params);
  cv::vector<cv::KeyPoint> myBlobs;
  myBlobDetector.detect(img, myBlobs);

  cv::Mat blobImg;
  cv::drawKeypoints(img, myBlobs, blobImg);

  std::cout << "plop" << std::endl;
  for(cv::vector<cv::KeyPoint>::iterator blobIterator = myBlobs.begin(); blobIterator != myBlobs.end(); blobIterator++){
   std::cout << "size of blob is: " << blobIterator->size << std::endl;
   std::cout << "point is at: " << blobIterator->pt.x << " " << blobIterator->pt.y << std::endl;
   cv::circle(blobImg,cv::Point(blobIterator->pt.x,blobIterator->pt.y),blobIterator->size,cv::Scalar(255,0,0),2,8,0);
  }
  cv::imshow("Blobs", blobImg);
  * */


  //ros
  //while(cv::waitKey(100)==-1 && ros::ok());
  ros::spin();
  return 0;
}
