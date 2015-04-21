#include "ros/ros.h"
 #include "path_finding.h"
  #include "path_finding/PathFinding.h"


/****************Compute TF********************/
void PathFinding::computeTF(std::string robot_id)
{
  
   tf::StampedTransform tf_robot;

    try
    {  
        tf_listener_.lookupTransform("/map", robot_id + "/base_link", ros::Time(0), tf_robot); /***/
                                                                   

    }
    catch (tf::TransformException ex)
    {   
        ROS_ERROR("Error reading TF");
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();

    }
    double x_o = tf_robot.getOrigin().x();
    double y_o = tf_robot.getOrigin().y();
    double yaw_angle_o = tf::getYaw(tf_robot.getRotation()); //get yaw-angle in radian
    
    //ROS_INFO_STREAM("TF robot : " << x_o<< "  " << y_o << "  "<< yaw_angle_o);
    // compute where the robot is in a grid corresponding to the /map frame 
    x_robot_origin = (int)( ( - x_map_origin + x_o) / map_resolution); // convert meter in pixel 
    y_robot_origin = (int)( ( - y_map_origin - y_o) / map_resolution); 

    theta_robot_origin= yaw_angle_o; // initial yaw-angle in radian
}



//******************map origine point once***********************************/
void PathFinding::map_origine_point(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{   
    map_resolution=msg->info.resolution;
    x_map_origin = msg->info.origin.position.x;
    y_map_origin = msg->info.origin.position.y;
    z_map_origin=tf::getYaw(msg->info.origin.orientation); // in radian


    //convert OccupancyGrid message into a map
    int count=0;
    map_received= cv::Mat::zeros(msg->info.width,msg->info.height,CV_32F);
    for (int i=0; i<map_received.rows;i++)
    {
        for (int j=0; j<map_received.cols;j++)
        {   
          
              // determine the index into the map data
              int mapI = j + ((map_received.cols - i - 1) * map_received.rows);
              // determine the value
              float data =(float) msg->data[mapI];
              if (data == 100.0) {

                   map_received.at<float>(i,j) = 0;
                   map_received.at<uchar>(i,j) = (uchar)map_received.at<float>(i,j);
    
               } else if (data == 0.0) {
               
                   map_received.at<float>(i,j) = 255;
                   map_received.at<uchar>(i,j) = (uchar)map_received.at<float>(i,j);


              } else {
                 
                   map_received.at<float>(i,j) = 0; 
                   map_received.at<uchar>(i,j) = (uchar)map_received.at<float>(i,j);

             }
              }
          
        }

}


//********* Algorithm *******************
std::vector<Node*> PathFinding::algorithm()
{

    //cv::Mat 
    map= map_received.clone();
    Node tree(x_robot_origin,y_robot_origin);
    
  
 	_rrt(&tree, NUMBER_OF_POINTS, map, x_robot_des, y_robot_des); 

    /**Add of the destination point**/
    Node end(x_robot_des,y_robot_des);


    std::vector<Node*> path = path_smoothing(rrt_path(&end,&tree), &map);
    draw_path(path,map);


 
    return path;

}


bool PathFinding::serviceCB(path_finding::PathFinding::Request  &req,
          path_finding::PathFinding::Response &res)
{

    ros::Time second=ros::Time::now();
    //ROS_INFO_STREAM(req.target.x<<" "<<req.target.y);
    x_robot_des = (-x_map_origin + req.target.x)/map_resolution;
    y_robot_des =(-y_map_origin - req.target.y)/map_resolution;

    theta_robot_des =req.target.theta; // yaw-angle in radian

    computeTF(req.robot_id.data);

    std::vector<Node*> path_bis;


        path_bis = algorithm();

    /***get coordinates of the destination point of /map in the /map frame***/
    res.path.header.frame_id ="/map" ;
    res.path.header.stamp = ros::Time();

    /***path publication***/
    ROS_INFO_STREAM("PATH_BIS_SIZE "<<" "<<path_bis.size());

    for( std::vector< Node* >::reverse_iterator rit_node = path_bis.rbegin() + 1; rit_node!=path_bis.rend(); ++rit_node)
    {

        geometry_msgs::Pose2D p;
 
        if( rit_node == path_bis.rbegin())
        {
            p.x = (*rit_node)->x * map_resolution ; // convert pixel in meter
            p.y = (*rit_node)->y * map_resolution ;

        }

        else
        {
        	if( (rit_node+1) == path_bis.rend())
            {
                p.x =   req.target.x ; // convert pixel in meter
                p.y =  req.target.y ;

            }
        	else
        	{
            p.x = ((*rit_node)->x)  * map_resolution + x_map_origin ; // convert pixel in meter
            p.y = -((*rit_node)->y)  * map_resolution - y_map_origin;
        	}
            
        }

        if((rit_node + 1) != path_bis.rend() ) 
        {   

            dx =  (*rit_node)->x - (*(rit_node + 1))->x ; 
            dy =  -(*rit_node)->y + (*(rit_node + 1))->y;

            alpha = atan2(dy,dx);


            angle = alpha -theta_robot_origin + PI ;
            p.theta=angle;

        }

        else
        {
            //ROS_INFO_STREAM("Angle END#"<<angle*180/PI);
            p.theta=theta_robot_des;

        }
        res.path.poses.push_back(p);

    }

    time=ros::Time::now().toSec()-second.toSec();
    ROS_INFO_STREAM("Path_finding duration :"<<" "<<time);

    return true;


 }
 



int main(int argc, char **argv)
 {    
    ros::init(argc, argv, "path_finding_server");
    ros::NodeHandle n;
    PathFinding pf(n);
    /****get param*****/
    ros::ServiceServer service = n.advertiseService("path_finding",&PathFinding::serviceCB,&pf);

    ROS_INFO("Ready to compute path.");

    /***find resolution of the map***/
    ros::Subscriber origine = n.subscribe<nav_msgs::OccupancyGrid>("map", 1,&PathFinding::map_origine_point,&pf); 
   
    ros::Rate loop(LOOP_RATE);
    while(ros::ok())
    {   

      		ros::spinOnce();
		    loop.sleep();
	}  
     
    ros::spin();

    return 0;
 }



