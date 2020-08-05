// this node receive path info and publish to rviz
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include "tsp/draw.h"
#include "tsp/point.h"
#include <iostream>

Draw::Draw(ros::NodeHandle * nh, Point *points){
    marker_pub = nh->advertise<visualization_msgs::Marker>("visualization_marker", 10);
    path_sub = nh->subscribe("/path", 1000, &Draw::paint, this);
    this->points = points;
}

// path msg contains the node index in the path
void Draw::paint(const std_msgs::Int32MultiArray& msg){
    std::vector<int> index = msg.data;
    std::cout<<points[index[0]].x<<' '<<points[index[0]].y;
}

int main (int argc, char **argv)
{
    ros::init(argc, argv, "draw_tsp");
    ros::NodeHandle nh;
    int num_points;
    if (nh.getParam("NUM_POINTS", num_points))
    {
      ROS_INFO("Got param NUM_POINTS: %d", num_points);
    }
    else
    {
      ROS_ERROR("Failed to get param 'NUM_POINTS'");
    }
    Draw draw = Draw(&nh, points);
    ros::spin();
}