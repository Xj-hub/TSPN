#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include "draw.h"
#include "point.h"
#include "gtsp/set.h"
#include "util.h"
#include <iostream>

int main(int argc, char ** argv){
    std::vector<Set> sets = readSets();
    ros::init(argc, argv, "draw_gtsp");
    ros::NodeHandle n;
    ros::Rate r(0.5);
    ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 20);

    ROS_INFO("paint the sets");
    int num_set = sets.size();
    visualization_msgs::Marker nodes[num_set], neighbors[num_set];
    for(int i = 0; i < num_set; ++i){
        
        nodes[i].header.frame_id = "/gtsp_frame";
        nodes[i].header.stamp = ros::Time::now();
        nodes[i].ns = "draw";
        nodes[i].action = visualization_msgs::Marker::ADD;
        nodes[i].pose.orientation.w = 1.0;
        nodes[i].id = i;
        nodes[i].type = visualization_msgs::Marker::POINTS;
        nodes[i].scale.x = 0.3;
        nodes[i].scale.y = 0.3;
        nodes[i].color.g = 1.0f;
        nodes[i].color.a = 1.0;

        neighbors[i].header.frame_id = "/gtsp_frame";
        neighbors[i].header.stamp = ros::Time::now();
        neighbors[i].ns = "draw";
        neighbors[i].action = visualization_msgs::Marker::ADD;
        neighbors[i].pose.orientation.w = 1.0;
        neighbors[i].id = i + num_set;
        neighbors[i].type = visualization_msgs::Marker::LINE_STRIP;
        neighbors[i].scale.x = 0.1;
        // neighbors[i].scale.y = 1.0;
        neighbors[i].color.r = 1.0f;
        neighbors[i].color.a = 0.8;
        
        int num_points = sets[i].points.size();
        for(Point point: sets[i].points){
            geometry_msgs::Point p;
            p.x = point.x;
            p.y = point.y;
            p.z = 0;

            nodes[i].points.push_back(p);

        }
        if(num_points >=2){
            for(int j = 0; j < num_points; ++j){
                neighbors[i].points.push_back(nodes[i].points[j]);
            }
            neighbors[i].points.push_back(nodes[i].points[0]);
        }
        
    }
    while(ros::ok()){
        for (int i = 0; i < num_set; ++i){
            marker_pub.publish(nodes[i]);
            marker_pub.publish(neighbors[i]);
        }
        r.sleep();
    }  
}