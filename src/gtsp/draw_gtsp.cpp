#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include "tsp/GtspPath.h"
#include "draw.h"
#include "point.h"
#include "gtsp/set.h"
#include "util.h"
#include <iostream>

Draw::Draw(ros::NodeHandle * nh, std::vector<Set> sets){
    marker_pub = nh->advertise<visualization_msgs::Marker>("visualization_marker", 100);
    path_sub = nh->subscribe("/path", 1000, &Draw::paint_gtsp, this);
    this->sets = sets;
}

void Draw::paint_gtsp(const tsp::GtspPath& msg){
    std::vector<int> set_index = msg.set_index;
    std::vector<int> point_index = msg.point_index;
    ROS_INFO("paint the sets");
    int num_set = sets.size();
    visualization_msgs::Marker nodes[num_set], neighbors[num_set],edges;

    // draw all the sets
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
        neighbors[i].color.g = 1.0f;
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

    //draw path
    edges.header.frame_id = "/gtsp_frame";
    edges.header.stamp = ros::Time::now();
    edges.ns = "draw";
    edges.action = visualization_msgs::Marker::ADD;
    edges.pose.orientation.w = 1.0;
    edges.id = 1 + 2* num_set;
    edges.type = visualization_msgs::Marker::LINE_STRIP;
    edges.scale.x = 0.1;
    edges.color.r = 1.0f;
    edges.color.a = 0.8;
    for(int i = 0; i < num_set; ++i){  
        geometry_msgs::Point p;
        p.x = sets[set_index[i]].points[point_index[i]].x;
        p.y = sets[set_index[i]].points[point_index[i]].y;
        p.z = 0;
        edges.points.push_back(p);
    }
    geometry_msgs::Point p;
    p.x = sets[set_index[0]].points[point_index[0]].x;
    p.y = sets[set_index[0]].points[point_index[0]].y;
    p.z = 0;
    edges.points.push_back(p);

    
    for (int i = 0; i < num_set; ++i){
        marker_pub.publish(nodes[i]);
        marker_pub.publish(neighbors[i]);
    }
    marker_pub.publish(edges);
}

int main(int argc, char ** argv){
    std::vector<Set> sets = readSets();
    ros::init(argc, argv, "draw_gtsp");
    ros::NodeHandle nh;
    Draw draw = Draw(&nh, sets);
    ros::spin();
    
}