/*********Path Finding*******************/

#include "path_finding.h"

using namespace cv;
using namespace std;




int main(int argc, char** argv)
{
    ros::init(argc, argv, "path_finding_node");
	ros::NodeHandle n;

    PathFinding pf(n);

    //find resolution of the map
    ros::Subscriber origine = n.subscribe<nav_msgs::OccupancyGrid>("/map", 1,&PathFinding::map_origine_point,&pf); 
    //find destination point and orientation
    ros::Subscriber destination = n.subscribe<scenario_msgs::Scenario>("scenario", 1,&PathFinding::computePath,&pf); 



    ros::Rate loop(LOOP_RATE),rate(3);
    while(ros::ok())
    {   

            rate.sleep(); 
            if(pf.waitFormap==false)    
            pf.computeTF();
      		ros::spinOnce();
		    loop.sleep();
	}      
    
		    


 	return 0;
}
