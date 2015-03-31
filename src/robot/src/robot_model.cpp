#include "ros/ros.h"
#include "visualization_msgs/Marker.h"


ros::Publisher marker_pub;


int main(int argc, char **argv)
{
    ros::init(argc, argv, "robot_model_viz");
    ros::NodeHandle n;

    marker_pub = n.advertise<visualization_msgs::Marker>( "robot_model_rviz", 0);
    visualization_msgs::Marker marker;

    marker.header.frame_id = "/base_link";
    marker.header.stamp = ros::Time(0);
    marker.id = 1;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0475;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.345;
    marker.scale.y = 0.545;
    marker.scale.z = 0.15;
    marker.color.a = 0.8;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.lifetime = ros::Duration(0);
    ros::Rate loop(10);
    while (ros::ok())
    {
        marker_pub.publish(marker);
        loop.sleep();
    }
    return 0;
}
