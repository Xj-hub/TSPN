#include <iostream>
#include <sstream>
#include <ros/package.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int32MultiArray.h"

#include "tsp/point.h"
#include "tsp/line.h"
#include "tsp/rkga.h"
#include "tsp/util.h"
#include<fstream>

#define NUM_POINTS 30
// Point points[NUM_POINTS];


std::vector<Point> readPoints(){
    std::string cwd_path = ros::package::getPath("tsp");
    std::string points_file = cwd_path + "/config/com.txt";

    std::ifstream in;
    in.open(points_file);
    std::string s;
    std::vector<Point> points;
    int count = 0;
    while (getline(in, s)){
        //逐行读取数据并存于s中，直至数据全部读取
        float x,y;
        std::stringstream point_xy;
        point_xy.str(s);
        point_xy>>x;
        point_xy>>y;
        points.push_back(Point(x,y));
        std::cout<<x<<' '<<y<<count<<'\n';
        count ++;
    }
    return points;
}


std::vector<int> ga_main(std::vector<Point> &points){
    RKGA ga = RKGA(points, 0.35, 0.55, 0.30, 0.005, NUM_POINTS, 3000, 20);
    ga.initialize();
    int generation = 0;
    while(generation < ga.MaxGeneration){
        int selection_size = ga.select();
        int crossover_size = ga.crossover();
        generation ++;
    }
    //ga.printResult();
    return ga.calculatePath();
}

int main(int argc, char **argv){
    ros::init(argc, argv, "path_talker");
    ros::NodeHandle n;
    ros::Publisher chatter_pub = n.advertise<std_msgs::Int32MultiArray>("path", 1000);
    ros::Rate loop_rate(1);

    // to test GA first initialize points randomly
    std::vector<Point> points = readPoints();
    while(ros::ok()){
        std_msgs::Int32MultiArray msg;
        msg.data = ga_main(points);

        ROS_INFO("%d", msg.data[0]);
        chatter_pub.publish(msg);

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}