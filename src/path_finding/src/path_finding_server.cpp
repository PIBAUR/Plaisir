#include "ros/ros.h"
 #include "path_finding.h"
  #include "path_finding/PathFinding.h"
using namespace cv;


/****************Compute TF********************/
void PathFinding::computeTF()
{
  
   tf::StampedTransform tf_robot;
  
    try
    {  
        tf_listener_.lookupTransform("/map", "/robot01/base_link", ros::Time(0), tf_robot);
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
    x_robot_origin = (int)( ( - x_map_origin + x_o) / map_resolution); // convert meter in pixel 
    y_robot_origin = (int)( ( - y_map_origin - y_o) / map_resolution); 
    ROS_INFO_STREAM("Robot position  metre : " << x_o<<" | "<<y_o);
    ROS_INFO_STREAM("Robot position : " << x_robot_origin<<" | "<<y_robot_origin);
    theta_robot_origin= yaw_angle_o; // initial yaw-angle in radian
}



//******************map origine point once***********************************/
void PathFinding::map_origine_point(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{   
    map_resolution=msg->info.resolution;
    x_map_origin = msg->info.origin.position.x;
    y_map_origin = msg->info.origin.position.y;
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
vector<Node*> PathFinding::algorithm()
{

    Mat map= map_received.clone();
    Node tree(x_robot_origin,y_robot_origin);
    //tree.x = x_robot_origin; tree.y = y_robot_origin;
       std::srand(std::time(0));
     /*
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
*/
  
 	_rrt(&tree, NUMBER_OF_POINTS, map, x_robot_des, y_robot_des); 

    /**Add of the destination point**/
    Node end(x_robot_des,y_robot_des);
    //end.x =  x_robot_des; end.y =  y_robot_des;
   /*
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
*/
    ROS_INFO_STREAM("COMPUTING PATH FINDING FROM : " << tree.x << " | "<< tree.y << "   to   " << end.x << " | "<< end.y);
    vector<Node*> path = path_smoothing(rrt_path(&end,&tree), &map);
    draw_path(path,map);
      namedWindow( "Path", WINDOW_AUTOSIZE );// Create a window for display.
  imshow( "Path", map );                   // Show our image inside it.
  waitKey(2000); 

 
    return path;

}


bool PathFinding::serviceCB(path_finding::PathFinding::Request  &req,
          path_finding::PathFinding::Response &res)
{
    ros::Time second=ros::Time::now();
    x_robot_des =  (-x_map_origin + req.target.x)/map_resolution;
    y_robot_des =  (-y_map_origin - req.target.y)/map_resolution;
    theta_robot_des =req.target.theta; // yaw-angle in radian

    computeTF();

    std::vector<Node*> path_bis;

    while(path_bis.size() <=1)
    {
        path_bis = algorithm();


    }

    /***get coordinates of the destination point of /map in the /map frame***/
    res.path.header.frame_id ="/map" ;
    res.path.header.stamp = ros::Time();
    /***path publication***/
    ROS_INFO_STREAM("PATH_BIS_SIZE "<<" "<<path_bis.size());

    for( std::vector< Node* >::reverse_iterator rit_node = path_bis.rbegin(); rit_node!=path_bis.rend(); ++rit_node)
    {
        ROS_INFO_STREAM("POINT : " << (*rit_node)->x * map_resolution << " | " << (*rit_node)->x * map_resolution );
        geometry_msgs::Pose p;
        if( rit_node == path_bis.rbegin())
        {
            p.position.x = (*rit_node)->x * map_resolution ; // convert pixel in meter 
            p.position.y = (*rit_node)->y * map_resolution ; 
        }
        else
        {
            p.position.x = ((*rit_node)->x)  * map_resolution + x_map_origin ; // convert pixel in meter 
            p.position.y = -((*rit_node)->y)  * map_resolution - y_map_origin;             
            
        }
        //cout<<path_bis[i]->x  * map_resolution<<" "<<path_bis[i]->y * map_resolution<<endl;
        // Comppf.computeTF();ute angle for each position
        
        if( (rit_node + 1) != path_bis.rend() )
        {     

            dx = (*(rit_node + 1))->x - (*rit_node)->x; 
            dy = (*(rit_node + 1))->y - (*rit_node)->y;
            //dx =((*rit_node)->x - (*(rit_node +1))->x); 
            //dy =((*rit_node)->y - (*(rit_node +1))->y);
            ROS_INFO_STREAM("DY #"<<dy<<" "<<"DX #"<<dx);
            if((dx<0)||(dy<0))
            alpha = -atan2(dy,dx);
            else
            alpha = atan2(dy,dx);
            ROS_INFO_STREAM("Alpha #"<<alpha*180/PI);
            angle = alpha -theta_robot_origin;
            p.orientation=tf::createQuaternionMsgFromYaw(angle);
        }
        else 
        {
            angle =theta_robot_des-theta_robot_origin;
            ROS_INFO_STREAM("Alpha END#"<<alpha*180/PI);
            p.orientation=tf::createQuaternionMsgFromYaw(angle);
        }
        res.path.poses.push_back(p);
    }
    path_pub.publish(res.path);

    time=ros::Time::now().toSec()-second.toSec();
    ROS_INFO_STREAM("Path_finding duration :"<<" "<<time);

    return true;
 }
 



int main(int argc, char **argv)
 {
    ros::init(argc, argv, "path_finding");
    ros::NodeHandle n;
    PathFinding pf(n);

    ros::ServiceServer service = n.advertiseService("path_finding",&PathFinding::serviceCB,&pf);
    ROS_INFO("Ready to compute path.");

    /***find resolution of the map***/
    ros::Subscriber origine = n.subscribe<nav_msgs::OccupancyGrid>("map", 1,&PathFinding::map_origine_point,&pf); 

    ros::Rate loop(LOOP_RATE),rate(5);
    while(ros::ok())
    {   

            //rate.sleep(); 
            //if(pf.waitFormap==false)    
            //pf.computeTF();
      		ros::spinOnce();
		    loop.sleep();
	}  
   
    ros::spin();

    return 0;
 }

/*Functions*/

