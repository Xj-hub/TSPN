#include <iostream>
#include <time.h>    //reset random seed time(NULL)

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int32MultiArray.h"

#include "point.h"
#include "tsp/line.h"
#include "tsp/rkga.h"
#include "util.h"



std::vector<int> ga_main(RKGA & ga, std::vector<Point> &points){
    int generation = 0;
    while(generation < ga.MaxGeneration){
        int selection_size = ga.select();
        int crossover_size = ga.crossover();
        ga.mutate();
        ga.immigrate();
        generation ++;
    }
    ga.printResult();
    return ga.calculatePath();
}

int main(int argc, char **argv){
    ros::init(argc, argv, "path_talker");
    ros::NodeHandle n;
    ros::Publisher chatter_pub = n.advertise<std_msgs::Int32MultiArray>("/path", 1000);
    ros::Rate loop_rate(1);
    srand(time(NULL));
    // to test GA first initialize points randomly
    std::vector<Point> points = readPoints();
    // RKGA ga = RKGA(points, 0.35, 0.55, 0.30, 0.005, 3000, 20);
    RKGA ga = RKGA(points,0.35, 0.55, 0.30, 0.005, 1000, 200);
    ga.initialize();
    int count = 0;
    while(ros::ok()){
        std_msgs::Int32MultiArray msg;
        msg.data = ga_main(ga, points);
        chatter_pub.publish(msg);
        ros::spinOnce();
        std::cout<<count<<'\n';
        loop_rate.sleep();
        count ++;
    }
    return 0;
}