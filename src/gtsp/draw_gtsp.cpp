#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include "draw.h"
#include "tsp/point.h"
#include "gtsp/set.h"
#include "util.h"
#include <iostream>

Draw::Draw(ros::NodeHandle * nh, std::vector<Set> sets){
    marker_pub = nh->advertise<visualization_msgs::Marker>("visualization_marker", 10);
    path_sub = nh->subscribe("/path", 1000, &Draw::paint, this);
    this->points = points;
}