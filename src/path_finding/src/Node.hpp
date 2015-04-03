#ifndef NODE_HPP
#define NODE_HPP

#include <iostream>
#include <vector>
#include <ros/ros.h>

struct Node{
 double x;
 double y;
 double distFromRoot;
 double distFromGoal;
 Node* parent;
 std::vector<Node*> forest; 

 Node(): x(0.0),y(0.0), distFromRoot(0.0), distFromGoal(0.0){
  parent = NULL;
 }
 Node(double x0, double y0): x(x0), y(y0), distFromRoot(0.0), distFromGoal(0.0){
  parent = NULL;
 }
 
};


#endif
