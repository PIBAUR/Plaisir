#include "ros/ros.h"
 #include "path_finding.h"
  #include "services/PathFinding.h"
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
    cout<<x_robot_origin<<" "<<y_robot_origin<<endl;
    theta_robot_origin= yaw_angle_o; // initial yaw-angle in radian
     

   waitFormap=true;

}

//******************map origine point once***********************************/
void PathFinding::map_origine_point(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{   
    map_resolution=msg->info.resolution;
    z_map_origin=tf::getYaw(msg->info.origin.orientation); // in radian

    ROS_INFO_STREAM("z_map_origin"<<" "<<z_map_origin<<" "<<"resolution"<<map_resolution);

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
                   map_received.at<float>(i,j) = 100;
                   map_received.at<uchar>(i,j) = (uchar)map_received.at<float>(i,j);
             
              } else {
                   map_received.at<float>(i,j) = data; //change into 0
                    //map_received.at<float>(i,j) = 0; 
                   map_received.at<uchar>(i,j) = (uchar)map_received.at<float>(i,j);
              }
             
        }
 
     }
   /**resize the matrix**/
/*Mat dst;
resize(map_received, dst, Size(),count/2, count/2, INTER_CUBIC); // upscale 2x
// or map.rows/2
//resize(src, dst, Size(1024, 768), 0, 0, INTER_CUBIC); 
  for (int i=0; i<map_received.rows;i++)
    {
        for (int j=0; j<map_received.cols;j++)
        {  
             cv::Scalar c= map_received.at<uchar>(i,j);
            if( c[0]==100)
                    dst.push_back(100);
        }
    }
  imshow("dst",dst);
  waitKey(0);
*/
}


//********* Algorithm *******************
vector<Node*> PathFinding::algorithm()
{
  
    Node tree;

    Mat map= map_received;
     
    tree.x = x_robot_origin; tree.y = y_robot_origin;
    //std::cout << tree.x << " " << tree.y << std::endl;

    //std::cout << "Initializing tree" << std::endl;
    std::srand(std::time(0));
  
    while(not_in_free_space(&tree,map,Randvalue) || !pixel_test(tree,map,0,Randvalue))
    {
         //std::cout << "first point"<<endl;
        if(x_robot_des < 0)
        {
        tree.x=((std::rand()% Randvalue+1) -Randvalue);
            if(y_robot_des < 0)
            tree.y = ((std::rand()% Randvalue+1) -Randvalue);
            else
            tree.y = (std::rand()% Randvalue);
        }
        else {
       
        tree.x = (std::rand()% Randvalue); // configure with map size
        tree.y = (std::rand()% Randvalue); 
        }
       
    }
    //std::cout << tree.x << " " << tree.y << std::endl;
     //std::cout << "Building graph" << std::endl;
    //rtt: More there are points more the graph will be 
 	_rrt(&tree, NUMBER_OF_POINTS, map, x_robot_des, y_robot_des,Randvalue);  // Choose 6000 points

    //std::cout << "Drawing graph" << std::endl;    
 	//Display trees
    Mat m_bis; map.copyTo(m_bis);
    affiche_tree(&tree,&m_bis);
    //imshow("rrt",m_bis);
     //waitKey(0);
    //std::cout << "Drawing path solution" << std::endl;
    Node end;
  
    //Add of the destination point

    end.x =  x_robot_des; end.y =  y_robot_des;
    std::srand(std::time(0));
    while(not_in_free_space(&end,map,Randvalue) || !pixel_test(end,map,0,Randvalue)) //modif 
    {
        if(x_robot_des < 0)
        {
        end.x=((std::rand()% Randvalue+1)  -Randvalue);
            if(y_robot_des < 0)
            end.y = ((std::rand()% Randvalue+1)  -Randvalue);
            else
            end.y = (std::rand()% Randvalue);
        }
        else {
        end.x = (std::rand()% Randvalue); // configure with map size
        end.y = (std::rand()% Randvalue); 
        }
    }
    vector<Node*> path = path_smoothing(rrt_path(&end,&tree), &map);
    //std::cout <<end.x << " " << end.y << std::endl;
    draw_path(path,&map);
  
   
    return path;

}


bool PathFinding::add(services::PathFinding::Request  &req,
          services::PathFinding::Response &res)
 {
            //ros::NodeHandle n;
            req.target.x=req.a;
            req.target.x=req.b;
            req.target.theta=req.c;
            //PathFinding pf(n);   
            cout<<"OK server"<<endl; 
            ros::Time second=ros::Time::now();
            x_robot_des =  (req.target.x)/map_resolution;
            y_robot_des =  (req.target.y)/map_resolution;
            theta_robot_des =req.target.theta; // yaw-angle in radian
             
            //ROS_INFO_STREAM("Travel");
         
            vector<Node*> path_bis=algorithm();
            if(path_bis.size() <=1)
            vector<Node*> path_bis=algorithm();
		    geometry_msgs::PoseArray path_copy;
		    // get coordinates of the destination point of /map in the /map frame
		    //path_copy.header.frame_id ="/map" ;
		    //path_copy.header.stamp = ros::Time();
               res.path.header.frame_id ="/map" ;
		      res.path.header.stamp = ros::Time();
		    //path publication
		    ROS_INFO_STREAM("PATH_BIS_SIZE "<<" "<<path_bis.size());

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

               
			    //path_copy.poses.push_back(p);
                res.path.poses.push_back(p);
		    }
		    //res.path=path_copy;
		    path_pub.publish(res.path);
		    
            time=ros::Time::now().toSec()-second.toSec();
            ROS_INFO_STREAM("Path_finding duration :"<<" "<<time);

        
        //ROS_INFO("request: x=%ld, y=%ld", (long int)req.a, req.b, req.c);
   
   return true;
 }
 
/*
YourRqtPlugin::YourRqtPlugin()  : 
  rqt_gui_cpp::Plugin()
{
  setObjectName("Qt based service server"); // or whatever
}

void YourRqtPlugin::initPlugin(qt_gui_cpp::PluginContext& context)
{  
     <<.........................>>
         other GUI intialization stuff
     <<.........................>>
      // advertise your service in initPlugin
      // use getNodeHandle() to get a nodehandle reference
    sleep(10);
     service = getNodeHandle().advertiseService("path_finding", &YourRqtPlugin::add, this);
}

void YourRqtPlugin::shutdownPlugin()
{
    // shutdown service; otherwise seg faults possible if service is called after plugin was unloaded
    service.shutDown();
}

bool YourRqtPlugin::add(services::PathFinding::Request  &req,
                    services::PathFinding::Response &res)
{
      cout<<ok<<endl;

     return true;
}
*/
int main(int argc, char **argv)
 {
  ros::init(argc, argv, "path_finding");
   ros::NodeHandle n;
    PathFinding pf(n);
     ros::ServiceServer service = n.advertiseService("path_finding",&PathFinding::add,&pf);
        ROS_INFO("Ready to compute path.");
    //find resolution of the map
    ros::Subscriber origine = n.subscribe<nav_msgs::OccupancyGrid>("/map", 1,&PathFinding::map_origine_point,&pf); 
    //find destination point and orientation
    //ros::Subscriber destination = n.subscribe<scenario_msgs::Scenario>("scenario", 1,&PathFinding::computePath,&pf); 



    ros::Rate loop(LOOP_RATE),rate(3);
    while(ros::ok())
    {   

            rate.sleep(); 
            if(pf.waitFormap==false)    
            pf.computeTF();
      		ros::spinOnce();
		    loop.sleep();
	}  
   // service
  

 ros::spin();

   return 0;
 }

/*Functions*/

