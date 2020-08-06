#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include "tsp/draw.h"
#include "tsp/point.h"
#include "tsp/util.h"
#include <iostream>

int main(int argc, char ** argv){
    ros::init(argc, argv, "basic_shapes");
    ros::NodeHandle n;
    ros::Rate r(1);
    ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

    uint32_t shape = visualization_msgs::Marker::SPHERE;

    while(ros::ok()){
        visualization_msgs::Marker marker;

        marker.header.frame_id = "/tsp_frame";
        marker.header.stamp = ros::Time::now();

        marker.ns = "basic_shapes";
        marker.id = 0;

        marker.type = shape;

        marker.action = visualization_msgs::Marker::ADD;

        marker.pose.position.x = 0;
        marker.pose.position.y = 0;
        marker.pose.position.z = 0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;

        marker.scale.x = 1.0;
        marker.scale.y = 2.0;
        marker.scale.z = 0.1;

        marker.color.r = 0.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;
        marker.color.a = 0.5;

        marker.lifetime = ros::Duration();

        while(marker_pub.getNumSubscribers()<1){
            if(!ros::ok()){
                return 0;
            }
            ROS_WARN_ONCE("Please create a subscribe to the marker");
            sleep(1);
        }
        marker_pub.publish(marker);
        
        // switch(shape){
        //     case visualization_msgs::Marker::CUBE:
        //         shape = visualization_msgs::Marker::SPHERE;
        //         break;
        //     case visualization_msgs::Marker::SPHERE:
        //         shape = visualization_msgs::Marker::ARROW;
        //         break;
        //     case visualization_msgs::Marker::ARROW:
        //         shape = visualization_msgs::Marker::CYLINDER;
        //         break;
        //     case visualization_msgs::Marker::CYLINDER:
        //         shape = visualization_msgs::Marker::CUBE;
        //         break;
        // }

        r.sleep();
    }
}