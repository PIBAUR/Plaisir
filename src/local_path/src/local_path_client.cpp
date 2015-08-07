 #include "ros/ros.h"
 #include "ros/console.h"
 #include "local_path/LocalPath.h"
 #include "local_path.hpp"
  #include <geometry_msgs/Pose2D.h>
 #include <cstdlib>


 int main(int argc, char **argv)
{
   ros::init(argc, argv, "local_path_client");

  if (argc != 4)
  {

     ROS_ERROR("Error reading arguments");

   }
  else
   {
   ros::NodeHandle n;
   ros::ServiceClient client = n.serviceClient<local_path::LocalPath>("local_path",true);

   local_path::LocalPath srv;

   try
   {

   srv.request.target.x= atof(argv[1]);
   srv.request.target.y= atof(argv[2]);
   srv.request.target.theta= atof(argv[3]);
   //ROS_INFO_STREAM("Reading arguments"); 

   }
   catch ( const std::exception & e ) 
    { 
        ROS_ERROR("Failed allocate arguments"); 
        ROS_ERROR("%s",e.what());
        ros::Duration(1.0).sleep();
    
    }
  if(ros::service::waitForService("local_path",-1)&&client.call(srv))
    {

     ROS_INFO_STREAM("OK"); 

    }
  else
  {
     ROS_ERROR("Failed to call service path_finding");
  }

  }
   return 0;
 }





