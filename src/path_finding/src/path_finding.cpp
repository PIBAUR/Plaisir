/*********Path Finding*******************/

#include "path_finding.h"

using namespace cv;
using namespace std;

#define map_path "/home/artlab/catkin_ws/maps/sentier2.pgm"
#define map_path_reworked "/home/artlab/catkin_ws/maps/sentier2_reworked.pgm"
//#define map_resolution 0.050000

void PathFinding::map_origine_point(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{   
    //ros::Rate rate(msg->info.map_load_time.sec); // rate en hertz=1/time
    //rate.sleep(); // wait the time at which the map was loaded
    map_resolution=msg->info.resolution;
    x_map_origin=msg->info.origin.position.x; // in meters
    y_map_origin=msg->info.origin.position.y; 
    z_map_origin=msg->info.origin.position.z; // in radian

    ROS_INFO_STREAM("x_map_origin"<<" "<<x_map_origin<<" "<<"y_map_origin"<<" "<<y_map_origin<<"  "<<"z_map_origin"<<" "<<z_map_origin);

    //convert OccupancyGrid message into a map
    map_received= Mat::zeros(msg->info.width,msg->info.height,CV_32F);
    for (int i=0; i<map_received.rows;i++)
    {
        for (int j=0; j<map_received.cols;j++)
        {   
              // determine the index into the map data
              int mapI = j + ((map_received.cols - i - 1) * map_received.rows);
              // determine the value
              float data =(float) msg->data[mapI];
              if (data == 100.0) {
               map_received.at<float>(i,j) = data;
               map_received.at<uchar>(i,j) = (uchar)map_received.at<float>(i,j);
               } else if (data == 0.0) {
               map_received.at<float>(i,j) = 100;
               map_received.at<uchar>(i,j) = (uchar)map_received.at<float>(i,j);
              } else {
               map_received.at<float>(i,j) = data;
               map_received.at<uchar>(i,j) = (uchar)map_received.at<float>(i,j);
              }
             
        }
 
     }



}
//********************* Compute of the destination Point ********************************

void PathFinding::destination_point(const scenario_msgs::Scenario::ConstPtr& msg)
{
   
  
   if(msg->type == "travel") //Check the good scenario
        {
        x_robot_des =  scenario.target.x;
        y_robot_des =  scenario.target.y;
        theta_robot_des = scenario.target.theta;
         
        ROS_INFO_STREAM("Target :"<<x_robot_des<<" "<<y_robot_des<<"  "<<theta_robot_des);

        computeTF();
        
		vector<Node*> path_bis=algorithm();

		geometry_msgs::PoseArray path_copy;
		// get coordinates of the destination point of /map in the /map frame
		path_copy.header.frame_id = "/map";
		path_copy.header.stamp = ros::Time();

		//path publication
		ROS_INFO_STREAM("PATH_BIS_SIZE "<<" "<<path_bis.size());
		for(int i=0;i<path_bis.size()-1;++i)
		{
			geometry_msgs::Pose p;
            cout<<path_bis[0]<<endl;
			p.position.x = x_map_origin + path_bis[i]->x * map_resolution; // convert pixel in meter
			p.position.y = abs (y_map_origin + path_bis[i]->y * map_resolution);//
			p.position.z = z_map_origin + theta_robot_des; // z_map_origin en radians
			ROS_INFO_STREAM("PATH_BIS: "<<path_bis[i]->x<<" "<<path_bis[i]->y);
			ROS_INFO_STREAM("POSITION: "<<p.position.x<<" "<<p.position.y<<" "<<p.position.z);

			path_copy.poses.push_back(p);
		}
		ROS_INFO_STREAM("BEGIN PUBLISHING");
		path_pub.publish(path_copy);
		ROS_INFO_STREAM("END OF PUBLISH");
        }
        
     else return;


}
//********* Algorithm *******************
vector<Node*> PathFinding::algorithm()
{
    
    /***********CALCUL PATH************/
  
    Node tree(0,0);
    //Node tree(127,322);

    Mat map= map_received;
    // Display image
    imshow( "Image2", map);

    //tree.x = x_robot_origin; tree.y = y_robot_origin;
    tree.x = x_robot_origin; tree.y = y_robot_origin;
    std::cout << tree.x << " " << tree.y << std::endl;

    std::cout << "Initializing tree" << std::endl;
    std::srand(std::time(0));
    while(not_in_free_space(&tree,map) || !pixel_test(tree,map,0))
    {
        tree.x = std::rand()%map.rows; tree.y = std::rand()%map.cols; // configure with map size
    }
    

     std::cout << "Building graph" << std::endl;
     //rtt
    //More there are points more the graph will be 
 	_rrt(&tree, 6000, map); 
	// _rrt(&tree, 10000, map);
    //_rrt(&tree,20000, map);
    std::cout << "Drawing graph" << std::endl;    
 	//Display trees
    Mat m_bis; map.copyTo(m_bis);
    affiche_tree(&tree,&m_bis);
    //namedWindow( "RRT graph", WINDOW_AUTOSIZE );// Create a window for display.

    imshow( "RRT graph", m_bis );    
  
    std::cout << "Drawing path solution" << std::endl;
    Node end;
  
    //Add of the destination point

    end.x =  x_robot_des; end.y =  y_robot_des;
    while(not_in_free_space(&end,map))
    {
        end.x = std::rand()%map.rows, end.y = std::rand()%map.cols;
    }
    vector<Node*> path = path_smoothing(rrt_path(&end,&tree), &map);

    draw_path(path,&map);
  
    //namedWindow("Path", WINDOW_AUTOSIZE );// Create a window for display.
 
    imshow("Path", map );// Show our image inside it.
    //waitKey(2);
    imwrite(map_path_reworked, map);
   
    return path;

}


/****************Calcul TF********************/
void PathFinding::computeTF()
{

    // get coordinates of the origin of /base_link in the /map frame
    geometry_msgs::PointStamped bl;
    bl.header.frame_id = "/robot01/base_link";
    bl.header.stamp = ros::Time();
    bl.point.x = 0.0;
    bl.point.y = 0.0;
    bl.point.z = 0.0;
    geometry_msgs::PointStamped robot;

    try {
    tf_listener_.transformPoint("/map", bl, robot);
    } catch (tf::TransformException& ex) {
    }

    double x_o = robot.point.x;
    double y_o = robot.point.y;
    ROS_INFO_STREAM("Target : x_robot_originTF"<<" "<< x_o<<"  "<<"y_robot_originTF"<<" "<< y_o);
    // compute where the robot is in a grid corresponding to the /map frame 

    x_robot_origin = (x_map_origin*map_resolution) + (x_o / map_resolution); // convert pixel in meter
    y_robot_origin = (y_map_origin*map_resolution) + (y_o / map_resolution); //
    theta_robot_origin=z_map_origin + robot.point.z; 
    
    ROS_INFO_STREAM("Target : x_origin"<<" "<<x_robot_origin<<"  "<<"y_origin"<<" "<<y_robot_origin<<"  "<<"theta_origin"<<robot.point.z);


}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "path_finding_node");
	ros::NodeHandle n;

    /******Publishers****************/
    //ros::Publisher path_pub = n.advertise<geometry_msgs::PoseArray>("path", 1);
    
    PathFinding pf(n);
    /*** get rosparam ***/

     

    /*****Subscribers****************/
    //find origin point in the map
    ros::Subscriber origine = n.subscribe<nav_msgs::OccupancyGrid>("/map", 1,&PathFinding::map_origine_point,&pf); 
    //find destination point and orientation
    ros::Subscriber destination = n.subscribe<scenario_msgs::Scenario>("scenario", 1,&PathFinding::destination_point,&pf); 

    ros::Rate loop(1);
    while(ros::ok())
    {
       // pf.spinOnce();
		ros::spinOnce();
		loop.sleep();
	}
	//ros::spin();

 	return 0;
}
