/*********Path Finding*******************/

#include "path_finding.h"

using namespace cv;
using namespace std;

/****************Compute TF********************/
void PathFinding::computeTF()
{

   tf::StampedTransform tf_robot;
  
    try
    {
          
        tf_listener_.lookupTransform("/map", "base_link", ros::Time(0), tf_robot);
    }
    catch (tf::TransformException ex)
    {   
        
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
        return;
    }
    double x_o = tf_robot.getOrigin().x();
    double y_o = tf_robot.getOrigin().y();
    double yaw_angle_o = tf::getYaw(tf_robot.getRotation()); //get yaw-angle in radian
    

    // compute where the robot is in a grid corresponding to the /map frame 
    x_robot_origin = (int) (x_o / map_resolution); // convert meter in pixel 
    y_robot_origin = (int) (y_o / map_resolution); 
    theta_robot_origin= yaw_angle_o; // initial yaw-angle in radian
    

   waitFormap=true;

}

//******************map origine point once***********************************/
void PathFinding::map_origine_point(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{   
    map_resolution=msg->info.resolution;
    z_map_origin=tf::getYaw(msg->info.origin.orientation); // in radian

    //ROS_INFO_STREAM("z_map_origin"<<" "<<z_map_origin<<" "<<"resolution"<<map_resolution);

    //convert OccupancyGrid message into a map
    int count=0;
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
                   map_received.at<float>(i,j) = 0;
                   map_received.at<uchar>(i,j) = (uchar)map_received.at<float>(i,j);
               } else if (data == 0.0) {
                   count++;
                   map_received.at<float>(i,j) = 255;
                   map_received.at<uchar>(i,j) = (uchar)map_received.at<float>(i,j);
             
              } else {
                   map_received.at<float>(i,j) = 0;
                   map_received.at<uchar>(i,j) = (uchar)map_received.at<float>(i,j);
              }
             
        }
 
     }

}
//********************* Compute of the destination Point ********************************

void PathFinding::computePath(const scenario_msgs::Scenario::ConstPtr& msg)
{

        if(msg->type == "travel") //Check the good scenario
        {   
            
         
            ros::Time second=ros::Time::now();
            x_robot_des =  (msg->target.x)/map_resolution;
            y_robot_des =  (msg->target.y)/map_resolution;
            theta_robot_des =msg->target.theta; // yaw-angle in radian
             
            //ROS_INFO_STREAM("Travel");
         
            vector<Node*> path_bis=algorithm();
            if(path_bis.size() <=1)
            vector<Node*> path_bis=algorithm();
		    geometry_msgs::PoseArray path_copy;
		    // get coordinates of the destination point of /map in the /map frame
		    path_copy.header.frame_id = "/map";
		    path_copy.header.stamp = ros::Time();
		    //path publication
		    //ROS_INFO_STREAM("PATH_BIS_SIZE "<<" "<<path_bis.size());

		    for(int i=path_bis.size()-1;i>=0;--i)
		    {
			    geometry_msgs::Pose p;

			    p.position.x =  path_bis[i]->x * map_resolution; // convert pixel in meter 
			    p.position.y = path_bis[i]->y * map_resolution; 
                
                // Compute angle for each position
                
                if(i!= 0)
                {              
                    dx = path_bis[i-1]->x - path_bis[i]->x; 
                    dy = path_bis[i-1]->y - path_bis[i]->y;
                    du  =sqrt(dx*dx+dy*dy);
                    alpha = atan2(dy,dx);
                    angle = alpha-theta_robot_origin;
                    p.orientation=tf::createQuaternionMsgFromYaw(angle);
                }
                else 
                {
                    angle =theta_robot_des-theta_robot_origin;
                    p.orientation=tf::createQuaternionMsgFromYaw(angle);
                }

               
			    path_copy.poses.push_back(p);
		    }
		   
		    path_pub.publish(path_copy);
		    
            time=ros::Time::now().toSec()-second.toSec();
            ROS_INFO_STREAM("Path_finding duration :"<<" "<<time);
          

        }
        
         else 
        {
            ROS_INFO_STREAM("NO SCENARIO RECEIVED");
            return;
        }

}




//********* Algorithm *******************
vector<Node*> PathFinding::algorithm()
{
  
    Mat map= map_received.clone();
    Node tree(x_robot_origin,y_robot_origin);
    //tree.x = x_robot_origin; tree.y = y_robot_origin;
       std::srand(std::time(0));
     while(not_in_free_space(&tree,map) || !pixel_test(tree,map,0))
        {
      
            if((x_robot_des < 0)&&(y_robot_des < 0))
            {
            tree.x=((std::rand()% map.rows+1) -map.rows);
            tree.y = ((std::rand()% map.cols+1) -map.cols);
            }
            else if((x_robot_des < 0)&&(y_robot_des >=0)){
            tree.x=((std::rand()% map.rows+1) -map.rows);
            tree.y = (std::rand()% map.cols); 
            }
            else if((x_robot_des >= 0)&&(y_robot_des < 0)){
            tree.x=(std::rand()% map.cols); 
            tree.y = ((std::rand()% map.cols+1) -map.cols);
            }
            else if((x_robot_des >= 0)&&(y_robot_des >= 0)){
            tree.x=(std::rand()% map.cols); 
            tree.y =(std::rand()% map.cols);
            }
           
        }
   
  
 	_rrt(&tree, NUMBER_OF_POINTS, map, x_robot_des, y_robot_des); 
    
    /**Add of the destination point**/
    Node end(x_robot_des,y_robot_des);
    //end.x =  x_robot_des; end.y =  y_robot_des;
    while(not_in_free_space(&end,map))
    {

        if((x_robot_des < 0)&&(y_robot_des < 0))
        {
        end.x=((std::rand()% map.rows+1) -map.rows);
        end.y = ((std::rand()% map.cols+1) -map.cols);
        }
        else if((x_robot_des < 0)&&(y_robot_des >=0)){
        end.x=((std::rand()% map.rows+1) -map.rows);
        end.y = (std::rand()% map.cols); 
        }
        else if((x_robot_des >= 0)&&(y_robot_des < 0)){
        end.x=(std::rand()% map.cols); 
        end.y = ((std::rand()% map.cols+1) -map.cols);
        }
        else if((x_robot_des >= 0)&&(y_robot_des >= 0)){
        end.x=(std::rand()% map.cols); 
        end.y =(std::rand()% map.cols);
        }
       
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

