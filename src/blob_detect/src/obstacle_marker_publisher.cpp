#include "ros/ros.h"
#include "scenario_msgs/ObstacleArray.h"
#include "visualization_msgs/MarkerArray.h"


ros::Publisher marker_pub;


void obstacleCB(const scenario_msgs::ObstacleArray& msg)
{
    visualization_msgs::MarkerArray markers_msg;

    scenario_msgs::ObstacleArray obstacles_msg= msg;

    for( std::vector< scenario_msgs::Obstacle >::iterator it_obs = obstacles_msg.obstacles.begin(); it_obs != obstacles_msg.obstacles.end(); it_obs+=1 )
    {
        visualization_msgs::Marker marker;

        marker.header.frame_id = "/map";
        marker.header.stamp = ros::Time(0);
        marker.id = it_obs->id;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = it_obs->x;
        marker.pose.position.y = it_obs->y;
        marker.pose.position.z = 0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 2* it_obs->radius;
        marker.scale.y = 2* it_obs->radius;
        marker.scale.z = 2* it_obs->radius;
        marker.color.a = 0.6;
        marker.color.r = 0.0;
        marker.color.g = 0.0;
        marker.color.b = 1.0;
        marker.lifetime = ros::Duration(0);
        markers_msg.markers.push_back(marker);
    }

    marker_pub.publish(markers_msg);
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "obstacle_marker_publisher");
    ros::NodeHandle n;

    marker_pub = n.advertise<visualization_msgs::MarkerArray>( "visualization_marker", 0);
    ros::Subscriber obstacle_sub = n.subscribe("obstacles", 1, obstacleCB);

    while (ros::ok())
    {
        ros::spin();
    }
    return 0;
}
