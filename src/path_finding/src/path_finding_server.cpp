#include "ros/ros.h"
#include "path_finding.h"
#include "path_finding/PathFinding.h"

/*
cv::Mat traitement_image(cv::Mat &map){
  // Gris -> Noir
  for(int i = 0; i < map.rows; i++){
    for(int j = 0; j < map.cols; j++){
      cv::Scalar c = map.at<uchar>(i,j);
      if(c[0] < 255)
        map.at<uchar>(i,j) = 0;
    }
  }
  cv::Mat dst, dst_erode;
  // Erosion/Dilatation
  cv::dilate(map,dst,cv::Mat(), cv::Point(-1,-1) ,5);
  cv::erode(dst, dst_erode,cv::Mat(), cv::Point(-1,-1), 5);

  return dst_erode;
}
*/
/****************Compute TF********************/
void PathFinding::computeTF(std::string robot_id)
{

    tf::StampedTransform tf_robot;
    ROS_INFO_STREAM("Id robot :\t"<<robot_id);
    try
    {
        tf_listener_.lookupTransform("/map", robot_id + "/base_link", ros::Time(0), tf_robot); /***/
        //tf_listener_.lookupTransform("/map", "robot01/base_link", ros::Time(0), tf_robot);
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

    ROS_INFO_STREAM("ROBOT ID" <<robot_id);
    //ROS_INFO_STREAM("TF robot : " << x_o<< "  " << y_o << "  "<< yaw_angle_o);
    // compute where the robot is in a grid corresponding to the /map frame
    //x_robot_origin = (int)( ( - x_map_origin + x_o) / map_resolution); // convert meter in pixel
    //y_robot_origin = (int)( ( - y_map_origin - y_o) / map_resolution);
    x_robot_origin = -(x_map_origin - x_o)/map_resolution;
    y_robot_origin = map_received.rows+(y_map_origin - y_o)/map_resolution;
    theta_robot_origin= yaw_angle_o; // initial yaw-angle in radian
}



//******************map origine point once***********************************/
void PathFinding::map_origine_point(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    //sens de la carte : de la gauche vers la droite
    map_resolution=msg->info.resolution;
    x_map_origin = msg->info.origin.position.x;
    y_map_origin = msg->info.origin.position.y;
    //z_map_origin=tf::getYaw(msg->info.origin.orientation); // in radian
    ROS_INFO_STREAM("Map position" <<" "<<x_map_origin<<";"<<y_map_origin);
    ROS_INFO_STREAM("Map size" <<" "<<msg->info.width<<"x"<<msg->info.height);
    //ROS_INFO_STREAM("Map orientation" <<" "<<z_map_origin);

    //convert OccupancyGrid message into a map
    int count=0;
    map_received= cv::Mat::zeros(msg->info.width,msg->info.height,CV_32F);
    for (int i=0; i<map_received.rows;i++)
    //for (int i= map_received.cols-1; i>=0 ;i--)
    {
        for (int j=0; j<map_received.cols;j++)
        //for (int j= map_received.cols-1; j>=0 ;j--)
        {
            // determine the index into the map data
            int mapI = j + ((map_received.cols - i - 1) * map_received.rows);
            //int mapI = j + ((map_received.rows + i - 1) * map_received.cols);
            // determine the value
            float data =(float) msg->data[mapI];
            if (data == 100.0)
            {
                map_received.at<float>(i,j) = 0;
                //map_received.at<uchar>(i,j) = (uchar)map_received.at<float>(i,j);
            }
            else if (data == 0.0)
            {
                map_received.at<float>(i,j) = 255;
                //map_received.at<uchar>(i,j) = (uchar)map_received.at<float>(i,j);
            }
            else
            {
                map_received.at<float>(i,j) = 0;
                //map_received.at<uchar>(i,j) = (uchar)map_received.at<float>(i,j);

            }
        }

    }

    x_map_pixel = -x_map_origin/map_resolution;
    y_map_pixel = map_received.rows+y_map_origin/map_resolution;

    cv::Mat map_before;
    map_before = map_received.clone();
    //cv::circle(map_before, cv::Point(x_robot_des,y_robot_des), 3, 0, 2, 8,0);
    //cv::circle(map_before, cv::Point(x_robot_origin,y_robot_origin), 6, 0, 2, 8,0);
    std::cout<<"Map origin in pixel : "<< x_map_pixel <<"|"<<y_map_pixel<<std::endl;
    cv::circle(map_before, cv::Point( x_map_pixel,y_map_pixel), 9, 0, 2, 8,0);
    cv::imshow("map_ori",map_before);
    cv::waitKey(20);
}


//********* Algorithm *******************
std::vector<Node*> PathFinding::algorithm()
{
    ROS_INFO_STREAM("X-Y Origin" <<" "<<x_robot_origin<<" "<<y_robot_origin);
    ROS_INFO_STREAM("X-Y Dest" <<" "<<x_robot_des<<" "<<y_robot_des);


    map= map_received.clone();
    cv::Mat map_before;
    map_before = map_received.clone();
    //map= cv::imread("/home/serveur/catkin_ws/maps/last_map.pgm", CV_LOAD_IMAGE_GRAYSCALE);
    //map = traitement_image(map);

    cv::imshow("map",map);
    cv::waitKey(20);


    Node tree(x_robot_origin,y_robot_origin);

    //std::cout<<"x_origin"<<" "<<x_robot_origin<<" "<<"y_origin"<<"  "<<y_origin<<std::endl;

    int largeur_robot=(int) (diametre_robot/(2*100)/map_resolution)+1; //conversion in meter
    int distance_detection= (int) (distance_obstacle_detection/100/map_resolution)+1;//conversion in meter
    int delta_rrt= (int) (deltaQ/100/map_resolution);//conversion in meter
    ROS_INFO_STREAM("Robot origin in pixel : "<< x_robot_origin <<"|"<<y_robot_origin<<std::endl;);
    //if(largeur_robot > 6 ) rrt_iterations_number+=5000; //security : 6 pixels
    ROS_INFO_STREAM("Largeur robot" <<"  "<<largeur_robot);
    ROS_INFO_STREAM("Distance detection" <<"  "<<distance_detection);
    ROS_INFO_STREAM("Delta Q" <<"  "<<delta_rrt);

    cv::circle(map_before, cv::Point(x_robot_des,y_robot_des), 3, 0, 2, 8,0);
    cv::circle(map_before, cv::Point(x_robot_origin,y_robot_origin), 6, 0, 2, 8,0);
    cv::circle(map_before, cv::Point( x_map_pixel,y_map_pixel), 9, 0, 2, 8,0);
    cv::imshow("map_before_rrt",map_before);
    cv::waitKey(200);

 	_rrt(&tree, rrt_iterations_number,map, x_robot_des, y_robot_des,largeur_robot, distance_detection,delta_rrt);
    //_rrt(&tree, 3000,map, x_robot_des, y_robot_des,largeur_robot, distance_detection,10);

	//********************************************************affiche arbre*****//
 	  cv::Mat m_bis; map.copyTo(m_bis);
 	  affiche_tree(&tree,&m_bis);
 	  //namedWindow( "RRT graph", cv::WINDOW_NORMAL );// Create a window for display.
 	  cv::imshow( "RRT graph", m_bis );
 	  cv::waitKey(20);
 	 std::cout << "Drawing path solution" << std::endl;
    //****************************************************************************

    /**Add of the destination point**/
    Node end(x_robot_des,y_robot_des);

    //cv:circle(map, cv::Point(x_map_origin,y_map_origin), 6, cv::Scalar(0,0,0), 2, 8,0);
    std::vector<Node*> path = path_smoothing(rrt_path(&end,&tree), &map, lissage_tolerance,lissage_force,lissage_coef,largeur_robot,distance_detection);
    draw_path(path,map);
    cv::imshow( "PATH", map );
    cv::waitKey(20);


    return path;

}


bool PathFinding::serviceCB(path_finding::PathFinding::Request  &req,
          path_finding::PathFinding::Response &res)
{

    ros::Time second=ros::Time::now();
    //ROS_INFO_STREAM(req.target.x<<" "<<req.target.y);
    //x_robot_des = (-x_map_origin + req.target.x)/map_resolution;
    //y_robot_des =(-y_map_origin - req.target.y)/map_resolution;
    x_robot_des = -(x_map_origin - req.target.x)/map_resolution;
    y_robot_des = map_received.rows+(y_map_origin - req.target.y)/map_resolution;

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
                p.y = map_resolution * (map_received.rows - ((*rit_node)->y)) + y_map_origin;
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
    if(path_bis.size()==1)
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
    ros::init(argc, argv, "path_finding_server");
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
    /***find resolution of the map***/
    ros::Subscriber origine = n.subscribe<nav_msgs::OccupancyGrid>("map", 1,&PathFinding::map_origine_point,&pf);


    ros::ServiceServer service = n.advertiseService("path_finding",&PathFinding::serviceCB,&pf);

    /***find resolution of the map***/
    //ros::Subscriber origine = n.subscribe<nav_msgs::OccupancyGrid>("map", 1,&PathFinding::map_origine_point,&pf);

    ROS_INFO("Ready to compute path.");

    ros::Rate loop(pf.loop_rate);
    while(ros::ok())
    {
        ros::spinOnce();
        loop.sleep();
	}

    ros::spin();

    return 0;
}


