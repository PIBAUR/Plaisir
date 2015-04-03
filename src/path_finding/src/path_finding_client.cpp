 #include "ros/ros.h"
 #include "path_finding/PathFinding.h"
 #include "path_finding.h"
  #include <geometry_msgs/Pose2D.h>
 #include <cstdlib>

using namespace std;

 int main(int argc, char **argv)
{
   ros::init(argc, argv, "path_finding_client");
   if (argc != 4)
  {
     ROS_INFO("usage: path_finding_client X Y Z");
     //return 1;
   }
   //else
      //ROS_INFO("ERROR read arguments");
   ros::NodeHandle n;
   ros::ServiceClient client = n.serviceClient<path_finding::PathFinding>("path_finding");
   //ros::Publisher path_pub=n.advertise<geometry_msgs::PoseArray>("path", 1);
   path_finding::PathFinding srv;
 
   
   srv.request.target.x=  atof(argv[1]);
   srv.request.target.y= atof(argv[2]);
   srv.request.target.theta=atof(argv[3]);
    cout<<srv.request.target.x<<" "<<srv.request.target.y<<" "<<srv.request.target.theta<<endl;
   if (client.call(srv))
   {
       cout<<"OK"<<endl;
       //cout<<srv.response.path[0]<<endl;
    //path_pub.publish();
   }
   else
  {
     ROS_ERROR("Failed to call service path_finding");
     //return 1;
  }
 
   return 0;
 }





