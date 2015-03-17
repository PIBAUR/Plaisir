/*********Path Finding*******************/

#include "path_finding.h"

using namespace cv;
using namespace std;


void PathFinding::map_origine_point(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{   
    ros::Rate rate(msg->info.map_load_time.sec); // rate en hertz=1/time
    rate.sleep(); // wait the time at which the map was loaded
    map_resolution=msg->info.resolution;
    z_map_origin=tf::getYaw(msg->info.origin.orientation); // in radian

    ROS_INFO_STREAM("z_map_origin"<<" "<<z_map_origin);

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

void PathFinding::computePath(const scenario_msgs::Scenario::ConstPtr& msg)
{
   
  int count=0;
  double  dx, dy,du, alpha, angle;
  double second=0;
   if(msg->type == "travel") //Check the good scenario
        {   
            second=ros::Time::now().toSec();
            ROS_INFO_STREAM("Time START:"<<" "<<ros::Time::now().toSec());
            count++;
            x_robot_des =  (msg->target.x)/map_resolution;
            y_robot_des =  (msg->target.y)/map_resolution;
            theta_robot_des =msg->target.theta; // yaw-angle in radian
             
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
      
			    p.position.x =  path_bis[i]->x * map_resolution; // convert pixel in meter 
			    p.position.y = path_bis[i]->y * map_resolution; 
                // Compute angle for each position
                if(i!= (path_bis.size()-2))
                {              
                    dx = path_bis[i+1]->x - path_bis[i]->x;
                    dy = path_bis[i+1]->y - path_bis[i]->y;
                    du  =sqrt(dx*dx+dy*dy);
                    alpha = atan2(dy,dx);
                    angle = alpha-theta_robot_origin;
                    while(angle<-PI)
                            angle+=2*PI;
                    while(angle>=PI)
                            angle-=2*PI;
                    angle*=K_TH;
                    p.orientation=tf::createQuaternionMsgFromYaw(angle);
                }
                else 
                {
                    angle =theta_robot_des-theta_robot_origin;
                    while(angle<-PI)
                            angle+=2*PI;
                    while(angle>=PI)
                            angle-=2*PI;
                    angle*=K_TH;
                    p.orientation=tf::createQuaternionMsgFromYaw(angle);
                }
                //Display info
                ROS_INFO_STREAM("YAW "<<abs(theta_robot_des - theta_robot_origin));
     			ROS_INFO_STREAM("PATH_BIS: "<<path_bis[i]->x<<" "<<path_bis[i]->y);
			    ROS_INFO_STREAM("POSITION: "<<p.position.x<<" "<<p.position.y);
                ROS_INFO_STREAM("Orientation "<<p.orientation.y<<" "<<p.orientation.z<<" "<<p.orientation.w);

			    path_copy.poses.push_back(p);
		    }
		    ROS_INFO_STREAM("BEGIN PUBLISHING");
		    path_pub.publish(path_copy);
		    ROS_INFO_STREAM("END OF PUBLISH");
            ROS_INFO_STREAM("Time END :"<<" "<<ros::Time::now().toSec());
            second=ros::Time::now().toSec()-second;
            ROS_INFO_STREAM("Path_finding duration :"<<" "<<second);
            if(count==1) exit(0);
        }
        
     else 
        {
            ROS_INFO_STREAM("NO TYPE RECEIVED");
            return;
        }


}


/****************Compute TF********************/
void PathFinding::computeTF()
{
   tf::StampedTransform tf_robot;
   
    try
    {
    	ROS_INFO_STREAM("avant angle");
        tf_listener_.lookupTransform("/map", "/robot01/base_link", ros::Time(0), tf_robot);
        
    }
    catch (tf::TransformException ex)
    {   
        
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
        return;
    }
    
    double yaw_angle_o = tf::getYaw(tf_robot.getRotation()); //get yaw-angle in radian
    while(yaw_angle_o<-PI)
        yaw_angle_o+=2*PI;
    while(yaw_angle_o>=PI)
        yaw_angle_o-=2*PI;
    yaw_angle_o*=K_TH;
    // get coordinates of the origin of /base_link in the /map frame
    geometry_msgs::PoseStamped bl;
    bl.header.frame_id = "/robot01/base_link";
    bl.header.stamp = ros::Time();
    geometry_msgs::PoseStamped robot;
    try {
        
        tf_listener_.transformPose("/map", bl, robot);
    } catch (tf::TransformException& ex) {
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
        return;
    }
    
    double x_o = robot.pose.position.x;
    double y_o = robot.pose.position.y;

    
    ROS_INFO_STREAM("Target : x_robot_originTF"<<" "<< x_o<<"  "<<"y_robot_originTF"<<" "<<y_o<<"z_robot_origin"<<"  "<<yaw_angle_o);

    // compute where the robot is in a grid corresponding to the /map frame 
    x_robot_origin = (int) (x_o / map_resolution); // convert meter in pixel 
    y_robot_origin = (int) (y_o / map_resolution); 
   ROS_INFO_STREAM("Target : angle"<<yaw_angle_o);
    theta_robot_origin= yaw_angle_o; // initial yaw-angle in radian
    
    ROS_INFO_STREAM("Target : x_origin"<<" "<<x_robot_origin<<"  "<<"y_origin"<<" "<<y_robot_origin<<"  "<<"theta_origin"<<theta_robot_origin);


}

//********* Algorithm *******************
vector<Node*> PathFinding::algorithm()
{
  
    Node tree;

    Mat map= map_received;

    tree.x = x_robot_origin; tree.y = y_robot_origin;
    std::cout << tree.x << " " << tree.y << std::endl;

    std::cout << "Initializing tree" << std::endl;
    std::srand(std::time(0));
    while(not_in_free_space(&tree,map) || !pixel_test(tree,map,0))
    {
        tree.x = (std::rand()% map.rows)-map.rows ; // configure with map size
        tree.y = (std::rand()% map.cols) -map.cols ; 
    }
     std::cout << "Building graph" << std::endl;
    //rtt: More there are points more the graph will be 
 	_rrt(&tree, NUMBER_OF_POINTS, map);  // Choose 6000 points

    std::cout << "Drawing graph" << std::endl;    
 	//Display trees
    Mat m_bis; map.copyTo(m_bis);
    affiche_tree(&tree,&m_bis);
  
    std::cout << "Drawing path solution" << std::endl;
    Node end;
  
    //Add of the destination point

    end.x =  x_robot_des; end.y =  y_robot_des;
    while(not_in_free_space(&end,map))
    {
        end.x =(std::rand()% map.rows)-map.rows ;//-map.rows ;
        end.y = (std::rand()% map.cols)-map.cols ; //-map.cols ;
    }
    vector<Node*> path = path_smoothing(rrt_path(&end,&tree), &map);

    draw_path(path,&map);
  
   
    return path;

}



int main(int argc, char** argv)
{
    ros::init(argc, argv, "path_finding_node");
	ros::NodeHandle n;

    PathFinding pf(n);

    //find destination point and orientation
    ros::Subscriber destination = n.subscribe<scenario_msgs::Scenario>("scenario", 1,&PathFinding::computePath,&pf); 
    //find resolution of the map
    ros::Subscriber origine = n.subscribe<nav_msgs::OccupancyGrid>("/map", 1,&PathFinding::map_origine_point,&pf); 


    ros::Rate loop(1);
    while(ros::ok())
    {
		ros::spinOnce();
		loop.sleep();
	}

 	return 0;
}
