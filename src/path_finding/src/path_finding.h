#ifndef PATH_FINDING_H
#define PATH_FINDING_H

/* INCLUDES */

#include <ros/ros.h>
//#include <geometry_msgs/PoseArray.h>
//#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Pose2D.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/OccupancyGrid.h>
#include "RRT.hpp"
#include <string>

//lib opencv
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "path_finding/PathFinding.h"
/* CONSTANTS */

#define SMOOTHING_STRENGTH 0.5
#define SMOOTHING_TOLERANCE 6
#define SMOOTHING_DATA_WEIGHT 0.5
#define NUMBER_OF_POINTS 20000
#define PI 3.14159265359
#define LOOP_RATE 50
#define ROBOT_DIAMETER 55
#define DISTANCE_OBSTACLE 14

 /*******Class Path_finding*******/


class PathFinding
{
//protected:
public:
    ros::NodeHandle nh_;
    //ros::Publisher pub;
    tf::TransformListener tf_listener_;
    cv::Mat map_received, map;
    double theta_robot_origin, theta_robot_des, z_map_origin;
    int x_robot_origin, y_robot_origin, x_robot_des, y_robot_des;
    double map_resolution;
    double x_map_origin, y_map_origin;
    double  dx, dy,du, alpha, angle,time;
    bool waitFormap;
    int rrt_iterations_number;
    double lissage_force, lissage_tolerance,lissage_coef,pi,loop_rate,diametre_robot, distance_obstacle_detection;

public:

    PathFinding(ros::NodeHandle nh): nh_(nh),theta_robot_origin(0.0), theta_robot_des(0.0), z_map_origin(0.0),x_robot_origin(0.0), y_robot_origin(0.0), x_robot_des(0.0), y_robot_des(0.0), map_resolution(1.0), dx(0.0), dy(0.0),du(0.0), alpha(0.0), angle(0.0), time(0.0)
    {
    	//pub=nh_.advertise<geometry_msgs::PoseArray>("/robot01/path_viz", 1);

    }
    ~PathFinding(){};
    void computeTF(std::string robot_id);
    std::vector<Node*> algorithm();
    void map_origine_point(const nav_msgs::OccupancyGrid::ConstPtr& msg);
    bool serviceCB(path_finding::PathFinding::Request  &req,path_finding::PathFinding::Response &res);
 
};



/* GLOBAL VARIABLES */


/* Headers*/

void affiche_tree(Node *q_i, cv::Mat* map);
void draw_path(std::vector<Node*> path, cv::Mat &map);
std::vector<Node*> rrt_path(Node *end, Node *tree);
std::vector<Node*> path_smoothing(std::vector<Node*> path, cv::Mat *map,double lissage_tolerance,double lissage_force,double lissage_coef ,int largeur_robot,int distance_detection);


/* FUNCTIONS */

void affiche_tree_rec(Node* q_i,cv::Mat* map){
  int n = q_i->forest.size(), i;
  
  
  for(i=0;i<n;i++){
      
    cv::Point point_q_i(q_i->x,q_i->y);
    cv::Point point_q_f(q_i->forest[i]->x,q_i->forest[i]->y);
    //cv::Scalar color_c(127,127,172);
    cv::Scalar color_c(0,0,255);
    line(*map,point_q_i,point_q_f,color_c);
    // recursive call on q_i's forest
    affiche_tree_rec(q_i->forest[i],map);
		
  }
  
  return; 
}

void affiche_tree(Node *tree,cv::Mat* map){	
  return affiche_tree_rec(tree,map);
}

void draw_path(std::vector<Node*> path, cv::Mat &map){

	int n = path.size();

	for(int i=0;i<(n-1);i++){
	
		cv::Point start(path[i]->x,path[i]->y);
		cv::Point end(path[i+1]->x,path[i+1]->y);
		cv::Scalar c(0,0,0);
		cv::line(map,start,end,c);
        //std::cout << "point #"<<i<<" : "<<path[i]->x<<" | "<<path[i]->y<<std::endl;
	
	}

}

std::vector<Node*> rrt_path(Node *end, Node *tree){
	
	Node *graph_end = closest_to(end, tree), *temp;
	std::vector<Node*> path;
	
	temp = graph_end;
	while(temp){
	
		path.push_back(temp);
		temp = temp->parent;
	
	}
	return path;
	
}

std::vector<Node*> path_smoothing(std::vector<Node*> path, cv::Mat *map,double lissage_tolerance,double lissage_force,double lissage_coef ,int largeur_robot,int distance_detection){

  int n = path.size();
  std::vector<Node*> newpath;
  Node *temp; 
  bool point_ok;
  double change;

  /* Making path deep copy */
  for(int i=0;i<n;i++){
    newpath.push_back(path[i]);
  }

  change = lissage_tolerance;
  while(change >= lissage_tolerance){

    change = 0;

    for(int i=1;i<(n-1);i++){
      
      /* updated, better point considered as OK (in map, in free space) */
      temp = new Node(newpath[i]->x,newpath[i]->y);
      
      temp->x += lissage_force*(path[i]->x - temp->x);
      temp->x +=lissage_coef*(newpath[i-1]->x + newpath[i+1]->x - 2*temp->x);
      
      temp->y += lissage_force*(path[i]->y - temp->y);
      temp->y += lissage_coef*(newpath[i-1]->y + newpath[i+1]->y - 2*temp->y);
      
      if(_collision_with_object(newpath[i-1],temp,*map,largeur_robot,distance_detection) && _collision_with_object(temp,newpath[i+1],*map,largeur_robot,distance_detection))
        { // if new path (both new trajectories) is OK we can change it
        change += sqrt((temp->x - newpath[i]->x)*(temp->x - newpath[i]->x)+(temp->y - newpath[i]->y)*(temp->y - newpath[i]->y)); // updating change
        delete newpath[i];
        newpath[i] = temp;
      } 
      else { // else we delete incorrect new node
        delete temp;
      }
      
    }

  }

  return newpath;
}

#endif

