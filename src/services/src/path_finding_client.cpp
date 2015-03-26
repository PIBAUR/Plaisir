   #include "ros/ros.h"
 #include "services/PathFinding.h"
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
   ros::NodeHandle n;
   ros::ServiceClient client = n.serviceClient<services::PathFinding>("path_finding");
   //ros::Publisher path_pub=n.advertise<geometry_msgs::PoseArray>("path", 1);
   services::PathFinding srv;
   srv.request.a= atoll(argv[1]); 
   srv.request.b= atoll(argv[2]);
   srv.request.c= atoll(argv[3]);
   
   srv.request.target.x=srv.request.a;
   srv.request.target.y=srv.request.b;
   srv.request.target.theta=srv.request.c;
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



