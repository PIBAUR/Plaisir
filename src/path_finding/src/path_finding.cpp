/*********Path Finding*******************/

#include "path_finding.h"

using namespace cv;
using namespace std;

//********************* Compute of the destination Point ********************************

void PathFinding::destination_point(const scenario_msgs::Scenario::ConstPtr& msg)
{
    std::string test_name = "travel"; //Check the good scenario
    if(msg->type==test_name)
    {
        x_robot_des =  msg->target.x;
        y_robot_des = msg->target.y;
        theta_robot_des = msg->target.theta;
         
        //ROS_WARN_STREAM("Target :"<<x_robot_des<<" "<<y_robot_des);
        ROS_INFO_STREAM("Target :"<<x_robot_des<<" "<<y_robot_des);
      }
     else return;
}
//********* Algorithm *******************
vector<Node*> PathFinding::algorithm()
{
    ros::Rate rate(1);
    /***********CALCUL PATH************/
  
    Node tree(0,0);
    //Node tree(127,322);
    //Chargement Map  
    Mat map_init, map;
    map = imread("/notre_bon_plaisir/maps/map.pgm", CV_LOAD_IMAGE_GRAYSCALE);
    map = traitement_image(map);
    // traitement et calcul
    tree.x = x_robot_origin; tree.y = y_robot_origin;

    std::cout << tree.x << " " << tree.y << std::endl;

    std::cout << "Initializing tree" << std::endl;
    std::srand(std::time(0));
    while(not_in_free_space(&tree,map) || !pixel_test(tree,map,0))
    {
        tree.x = std::rand()%map.rows; tree.y = std::rand()%map.cols; // configure with map size
    }


     std::cout << "Building graph" << std::endl;
    rate.sleep();
    //rtt
    //More there are points more the graph will be 
 	//_rrt(&tree, 4000, map); 
	// _rrt(&tree, 10000, map);
    _rrt(&tree,20000, map);
    std::cout << "Drawing graph" << std::endl;    
 	//Display trees
    Mat m_bis; map.copyTo(m_bis);
    affiche_tree(&tree,&m_bis);
    namedWindow( "RRT graph", WINDOW_AUTOSIZE );// Create a window for display.

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
  
    namedWindow("Path", WINDOW_AUTOSIZE );// Create a window for display.
 
    imshow("Path", map );// Show our image inside it.
    waitKey(0);            

    imwrite("/notre_bon_plaisir/maps/map_reworked.pgm",map);
   
    return path;

}

/****************Calcul TF********************/

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
        //return;
    }
    
    x_robot_origin=tf_robot.getOrigin().x();
    y_robot_origin=tf_robot.getOrigin().y();
    theta_robot_origin= tf::getYaw(tf_robot.getRotation());

    ROS_INFO_STREAM("Target : x_origin"<<" "<<x_robot_origin<<"  "<<"y_origin"<<" "<<y_robot_origin<<"  "<<"theta_origin"<<"  "<<theta_robot_origin);


}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "path_finding_node");
	ros::NodeHandle n;
    
    PathFinding pf(n);
    /*** get rosparam ***/

	
    /******Publishers****************/
    ros::Publisher path_pub = n.advertise<geometry_msgs::PoseArray>("path", 1);
     

    /*****Subscribers****************/
    
    //find destination point and orientation
    ros::Subscriber destination = n.subscribe<scenario_msgs::Scenario>("scenario", 1,&PathFinding::destination_point,&pf); 

    // frequency
    ros::Rate r(1); //1 second  

    //ROS Loop
    while (ros::ok())
    {
        r.sleep();
        pf.computeTF();
        vector<Node*> path_bis=pf.algorithm();
    
        int i= path_bis.size()- 1;
        geometry_msgs::PoseArray path_copy;
	    //path publication
        for(int i=0;i<path_bis.size();++i)
        {
            path_copy.poses[i].position.x=path_bis[i]->x;
            path_copy.poses[i].position.x=path_bis[i]->y;
            path_copy.poses[i].position.z=0; // not necessary
        }

    path_pub.publish(path_copy);

    ros::spinOnce();
    r.sleep();
    }

 	return 0;
}
