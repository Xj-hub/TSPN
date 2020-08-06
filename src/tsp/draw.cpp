// this node receive path info and publish to rviz
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include "draw.h"
#include "tsp/point.h"
#include "util.h"
#include <iostream>

Draw::Draw(ros::NodeHandle * nh, std::vector<Point> points){
    marker_pub = nh->advertise<visualization_msgs::Marker>("visualization_marker", 10);
    path_sub = nh->subscribe("/path", 1000, &Draw::paint, this);
    this->points = points;
}

// path msg contains the node index in the path
void Draw::paint(const std_msgs::Int32MultiArray& msg){
    std::vector<int> index = msg.data;
    ROS_INFO("paint the points");
    int num_points = points.size();
    
        visualization_msgs::Marker nodes, edges;
        nodes.header.frame_id = edges.header.frame_id = "/tsp_frame";
        nodes.header.stamp = edges.header.stamp = ros::Time::now();
        nodes.ns = edges.ns = "draw";
        nodes.action = edges.action = visualization_msgs::Marker::ADD;
        nodes.pose.orientation.w = edges.pose.orientation.w = 1.0;

        nodes.id = 0;
        edges.id = 1;

        nodes.type = visualization_msgs::Marker::POINTS;
        edges.type = visualization_msgs::Marker::LINE_STRIP;
        // POINTS markers use x and y scale for width/height respectively
        nodes.scale.x = 0.3;
        nodes.scale.y = 0.3;

        // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
        edges.scale.x = 0.2;

        // nodes are green
        nodes.color.g = 1.0f;
        nodes.color.a = 1.0;

        // edges is blue
        edges.color.r = 1.0;
        edges.color.a = 1.0;

        for(int i = 0; i < num_points; ++i){
            geometry_msgs::Point p;
            p.x = points[index[i]].x;
            p.y = points[index[i]].y;
            p.z = 0;

            nodes.points.push_back(p);
            edges.points.push_back(p);
        }
        marker_pub.publish(nodes);
        marker_pub.publish(edges);
        
}


int main (int argc, char **argv)
{
    ros::init(argc, argv, "draw_tsp");
    ros::NodeHandle nh;
    std::vector<Point> points = readPoints();
    Draw draw = Draw(&nh, points);
    ros::spin();
}