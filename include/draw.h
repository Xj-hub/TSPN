#ifndef DRAW_H
#define DRAW_H

#include <ros/ros.h>
#include "std_msgs/Int32MultiArray.h"
#include "tsp/point.h"
#include "gtsp/set.h"

class Draw{
private:
    std::vector<Point> points;
    std::vector<Set> sets;
    ros::Publisher marker_pub;
    ros::Subscriber path_sub;

public:
    Draw(ros::NodeHandle * nh, std::vector<Point> points);
    Draw(ros::NodeHandle * nh, std::vector<Set> sets);
    void paint(const std_msgs::Int32MultiArray& msg);
    void paint_gtsp(const std_msgs::Int32MultiArray& msg);
};
#endif