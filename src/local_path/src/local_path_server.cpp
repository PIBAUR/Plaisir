#include "ros/ros.h"
 #include "local_path.hpp"
  #include "local_path/LocalPath.h"
/****add functions*****/

static bool cpt=true;
void my_mouse_callback( int event, int x, int y, int flags,  void* ptr)
{
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

cv::Mat traitement_image(cv::Mat &map){
  // Gris -> Noir
  for(int i = 0; i < map.rows; i++){
    for(int j = 0; j < map.cols; j++){
      cv::Scalar c = map.at<uchar>(i,j);
      if(c[0] < 240)
        map.at<uchar>(i,j) = 0;
    }
  }
  cv::Mat dst, dst_erode;
  // Erosion/Dilatation
  cv::dilate(map,dst,cv::Mat(), cv::Point(-1,-1) ,5);
  cv::erode(dst, dst_erode,cv::Mat(), cv::Point(-1,-1), 5);

  return dst_erode;
}

//****************Inflation********************//
void LocalPath::inflationCB(const nav_msgs::GridCells::ConstPtr& msg)
{

}



//******************Obstacle pos***********************************//
void LocalPath::obstacleCB(const nav_msgs::GridCells::ConstPtr& msg)
{


}


//********* Algorithm *******************//
std::vector<Node*> LocalPath::algorithm()
{

    map= map_received.clone();

    Node tree(x_robot_origin,y_robot_origin);
    // Point pour la gestion du click :
    cv::Point p;

    //Chargement Map
    //Mat map_init, map;
    map = cv::imread("/home/serveur/catkin_ws/maps/artlab_new.pgm", CV_LOAD_IMAGE_GRAYSCALE);
    map = traitement_image(map);

    cv::namedWindow( "Selection point de départ", CV_WINDOW_NORMAL );
    while( cpt) {
      // free memory
      cvSetMouseCallback("Selection point de départ",my_mouse_callback,(void*) &p);
      cv::imshow("Selection point de départ",map);
      cv::waitKey(5);
    }
    cpt = true;

    tree.x = p.x; tree.y =  p.y;

    std::cout << "Initializing tree" << std::endl;
    //****************************

    std::cout<<"x_origin"<<" "<<tree.x<<" "<<"y_origin"<<"  "<<tree.y<<std::endl;




    //cc_rrt(&tree, NUMBER_OF_POINTS, map, x_robot_des, y_robot_des);
    cc_rrt(&tree, NUMBER_OF_POINTS, map, map.rows, map.cols);

    std::cout << "Drawing graph" << std::endl;
    //affiche arbres

    cv::Mat m_bis; map.copyTo(m_bis);
    affiche_tree(&tree,&m_bis);
    cv::namedWindow( "RRT graph", CV_WINDOW_NORMAL );// Create a window for display.
    cv::imshow( "RRT graph", m_bis );

    std::cout << "Drawing path solution" << std::endl;
    // Choix point de destination ...
    cpt = true;
    Node end(x_robot_des,y_robot_des);
    while(cpt) {
      // free memory
      cvSetMouseCallback("Selection point d'arrivée",my_mouse_callback,(void*) &p);
      cv::waitKey(5);
    }

    /**Add of the destination point**/
    end.x = p.x; end.y = p.y;
    std::cout<<"x_dest"<<" "<<end.x<<" "<<"y_dest"<<"  "<<end.y<<std::endl;
    std::vector<Node*> path = path_smoothing(rrt_path(&end,&tree), &map);
    draw_path(path,&map);

    cv::namedWindow( "Path", CV_WINDOW_NORMAL );// Create a window for display.
    cv::imshow( "Path", map);                   // Show our image inside it.
    cv::waitKey(0);

    cv::imwrite("/home/serveur/catkin_ws/maps/map_reworked.pgm",map);


    return path;

}


/****************Compute TF********************/
void LocalPath::computeTF()
{

   tf::StampedTransform tf_robot;
   std::string message =robot_num;
   message+="/base_link";

    try
    {
        tf_listener_.lookupTransform("/map",message, ros::Time(0), tf_robot); /***/


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
void LocalPath::map_origine_point(const nav_msgs::OccupancyGrid::ConstPtr& msg)
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



bool LocalPath::serviceCB(local_path::LocalPath::Request  &req,local_path::LocalPath::Response &res)
{

    ros::Time second=ros::Time::now();
    //ROS_INFO_STREAM(req.target.x<<" "<<req.target.y);
    x_robot_des = (-x_map_origin + req.target.x)/map_resolution;
    y_robot_des =(-y_map_origin - req.target.y)/map_resolution;

    theta_robot_des =req.target.theta; // yaw-angle in radian

    computeTF();

    std::vector<Node*> path_bis;


        path_bis = algorithm();

    /***get coordinates of the destination point of /map in the /map frame***/
    res.path.header.frame_id ="/map" ;
    res.path.header.stamp = ros::Time();
    path_copy.header.frame_id ="/map";
    path_copy.header.stamp = ros::Time();
    /***path publication***/
    ROS_INFO_STREAM("PATH_BIS_SIZE "<<" "<<path_bis.size());

    for( std::vector< Node* >::reverse_iterator rit_node = path_bis.rbegin() + 1; rit_node!=path_bis.rend(); ++rit_node)
    {

        geometry_msgs::Pose2D p;
        geometry_msgs::Pose p_test;
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
        p_test.position.x=p.x;
        p_test.position.y=p.y;
        if((rit_node + 1) != path_bis.rend() )
        {

            dx =  (*rit_node)->x - (*(rit_node + 1))->x ;
            dy =  -(*rit_node)->y + (*(rit_node + 1))->y;

            alpha = atan2(dy,dx);


            angle = alpha -theta_robot_origin + PI ;
            p.theta=angle;
            p_test.orientation=tf::createQuaternionMsgFromYaw(p.theta);

        }

        else
        {
            //ROS_INFO_STREAM("Angle END#"<<angle*180/PI);
            p.theta=theta_robot_des;
            p_test.orientation=tf::createQuaternionMsgFromYaw(p.theta);

        }
        res.path.poses.push_back(p);
        path_copy.poses.push_back(p_test);

    }
    pub.publish(path_copy);
    time=ros::Time::now().toSec()-second.toSec();
    ROS_INFO_STREAM("Path_finding duration :"<<" "<<time);

    return true;


 }




int main(int argc, char **argv)
 {
    ros::init(argc, argv, "local_path_server");
    ros::NodeHandle n;
    LocalPath pf(n);
    /****get param*****/
    n.param<std::string>("/robot_num",pf.robot_num,"/robot01");

    ROS_INFO("Ready to compute path.");
        /*****callback********/
    ros::ServiceServer service = n.advertiseService("local_path",&LocalPath::serviceCB,&pf);
    /***find resolution of the map***/
    //ros::Subscriber origine = n.subscribe<nav_msgs::OccupancyGrid>("map", 1,&LocalPath::map_origine_point,&pf);

    /***find local obstacle in the map***/
 ros::Subscriber obstacle_pos = n.subscribe<nav_msgs::GridCells>("obstaclemap", 1,&LocalPath::obstacleCB,&pf);
    /***find inflation in the map***/
ros::Subscriber inflation_pos = n.subscribe<nav_msgs::GridCells>("inflationmap", 1,&LocalPath::inflationCB,&pf);


    ros::Rate loop(LOOP_RATE);
    while(ros::ok())
    {

            ros::spinOnce();
            loop.sleep();
    }

    ros::spin();

    return 0;
 }


