#include "ros/ros.h"
#include "path_finding.h"
#include "test_param_path_finding/PathFinding.h"

bool cpt=true;
// Cv mouse

void my_mouse_callback( int event, int x, int y, int flags, void * ptr ){
  cv::Point * p = (cv::Point *) ptr;
  p->x=x;
  p->y=y;
  switch( event ){
    case CV_EVENT_LBUTTONDOWN:
    cpt=false;
    break;
    default:
    break;
  }
}



/****************Compute TF********************/
void PathFinding::computeTF(std::string robot_id)
{
  
   tf::StampedTransform tf_robot;

    try
    {  
        //tf_listener_.lookupTransform("/map", robot_id + "/base_link", ros::Time(0), tf_robot); /***/
        tf_listener_.lookupTransform("/map", "robot01/base_link", ros::Time(0), tf_robot);

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

    map= map_received.clone();
    //*******************************
    cv::Point p,p1;
    namedWindow( "Selection point de départ", cv::WINDOW_NORMAL );
     while( cpt) {
       // free memory
       cvSetMouseCallback("Selection point de départ",my_mouse_callback,(void*) &p);
       cv::imshow("Selection point de départ",map);
       cvWaitKey(5);
     }
     cpt = true;
    //***********************************
    //Node tree(x_robot_origin,y_robot_origin);
     Node tree(p.x,p.y);
     std::cout<<"init_x"<<" "<<p.x<<"  "<<"init_y"<<" "<<p.y<<std::endl;

    int largeur_robot=(int) (diametre_robot/(2*100)/map_resolution)+1; //conversion in meter
    int distance_detection= (int) (distance_obstacle_detection/100/map_resolution)+1;//conversion in meter
    int delta_rrt= (int) (deltaQ/100/map_resolution);//conversion in meter
    std::cout<<"delta_value"<<" "<<delta_rrt<<std::endl;

    if(largeur_robot > 6 ) rrt_iterations_number+=5000; //security : 6 pixels

    std::cout<<"iterations"<<" "<<rrt_iterations_number<<std::endl;

    _rrt(&tree, rrt_iterations_number,map,map.rows, map.cols,largeur_robot,distance_detection,delta_rrt);

 	//_rrt(&tree, rrt_iterations_number,map, map.rows, map.cols,largeur_robot,distance_detection,delta_rrt);
 	//********************************************************
 	//affiche arbres
 	  cv::Mat m_bis; map.copyTo(m_bis);
 	  affiche_tree(&tree,&m_bis);
 	  namedWindow( "RRT graph", cv::WINDOW_NORMAL );// Create a window for display.
 	  cv::imshow( "RRT graph", m_bis );
 	 std::cout << "Drawing path solution" << std::endl;
    //****************************************************************************
  	while(cpt) {
  	    // free memory
  	    cvSetMouseCallback("Selection point d'arrivée",my_mouse_callback,(void*) &p1);
  	    cvWaitKey(5);
  	  }

  	//********************************************************
    /**Add of the destination point**/
    Node end(p1.x,p1.y);
    std::cout<<"exit_x"<<" "<<p.x<<"  "<<"exit_y"<<" "<<p.y<<std::endl;

    std::vector<Node*> path = path_smoothing(rrt_path(&end,&tree), &map, lissage_tolerance,lissage_force,lissage_coef,largeur_robot,distance_detection);
    draw_path(path,map);

    //**********************************************************
   /*namedWindow( "Path", cv::WINDOW_NORMAL );// Create a window for display.
    cv::imshow( "Path", map);                   // Show our image inside it.
    cvWaitKey(5);
	*/
    //******************************************************

 
    return path;

}


bool PathFinding::serviceCB(test_param_path_finding::PathFinding::Request  &req,
		test_param_path_finding::PathFinding::Response &res)
{

    ros::Time second=ros::Time::now();
    //ROS_INFO_STREAM(req.target.x<<" "<<req.target.y);
    x_robot_des = (-x_map_origin + req.target.x)/map_resolution;
    y_robot_des =(-y_map_origin - req.target.y)/map_resolution;

    theta_robot_des =req.target.theta; // yaw-angle in radian

    computeTF(req.robot_id.data);

    std::vector<Node*> path_bis;


        path_bis = algorithm();
        //geometry_msgs::PoseArray path_copy;
    /***get coordinates of the destination point of /map in the /map frame***/
    res.path.header.frame_id ="/map" ;
    res.path.header.stamp = ros::Time();
    //path_copy.header.frame_id ="/map";
    //path_copy.header.stamp = ros::Time();
    /***path publication***/
    ROS_INFO_STREAM("PATH_BIS_SIZE "<<" "<<path_bis.size());

    for( std::vector< Node* >::reverse_iterator rit_node = path_bis.rbegin() + 1; rit_node!=path_bis.rend(); ++rit_node)
    {

        geometry_msgs::Pose2D p;
        //geometry_msgs::Pose p_test;
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
        //p_test.position.x=p.x;
        //p_test.position.y=p.y;

        if((rit_node + 1) != path_bis.rend() ) 
        {   

            dx =  (*rit_node)->x - (*(rit_node + 1))->x ; 
            dy =  -(*rit_node)->y + (*(rit_node + 1))->y;

            alpha = atan2(dy,dx);


            angle = alpha -theta_robot_origin + PI ;
            p.theta=angle;
            //p_test.orientation=tf::createQuaternionMsgFromYaw(p.theta);

        }

        else
        {

            //ROS_INFO_STREAM("Angle END#"<<angle*180/PI);
            p.theta=theta_robot_des;
            //p_test.orientation=tf::createQuaternionMsgFromYaw(p.theta);

        }
        res.path.poses.push_back(p);
        //path_copy.poses.push_back(p_test);
    }
    //pub.publish(path_copy);

    // Add a point if path size == 1
    //if(path_bis.size()==1)
    //0.5 m= 10 pixels
    if((abs(x_robot_des - x_robot_origin)< 0.5)||(abs(y_robot_des - y_robot_origin)< 0.5))
    {
    	 geometry_msgs::Pose2D p2;
         p2.x =   req.target.x ; // convert pixel in meter
         p2.y =  req.target.y ;

         p2.theta=theta_robot_des;

         res.path.poses.push_back(p2);
    }
    time=ros::Time::now().toSec()-second.toSec();
    ROS_INFO_STREAM("Path_finding duration :"<<" "<<time);

    return true;


 }
 



int main(int argc, char **argv)
 {    
    ros::init(argc, argv, "server_test");
    ros::NodeHandle n;
    PathFinding pf(n);
    /****get param *****/
   	n.param<double>("/rrt_iterations_number",pf.rrt_iterations_number,NUMBER_OF_POINTS);
    n.param<double>("/deltaQ",pf.deltaQ,DELTA); //(50 cm=10 pixels by default)
    n.param<double>("/lissage_force",pf.lissage_force,SMOOTHING_STRENGTH);
    n.param<double>("/lissage_tolerance",pf.lissage_tolerance,SMOOTHING_TOLERANCE);
    n.param<double>("/lissage_coef",pf.lissage_coef,SMOOTHING_DATA_WEIGHT);
    n.param<double>("/pi",pf.pi,PI);
    n.param<double>("/loop_rate",pf.loop_rate,LOOP_RATE);
    n.param<double>("/diametre_robot",pf.diametre_robot,ROBOT_DIAMETER );
    n.param<double>("/distance_obstacle_detection",pf.distance_obstacle_detection,DISTANCE_OBSTACLE);



    ros::ServiceServer service = n.advertiseService("server_test",&PathFinding::serviceCB,&pf);

    ROS_INFO("Ready to compute path.");

    /***find resolution of the map***/
    ros::Subscriber origine = n.subscribe<nav_msgs::OccupancyGrid>("map", 1,&PathFinding::map_origine_point,&pf); 
   
    ros::Rate loop(pf.loop_rate);
    while(ros::ok())
    {   

      		ros::spinOnce();
		    loop.sleep();
	}  
     
    ros::spin();

    return 0;
}


