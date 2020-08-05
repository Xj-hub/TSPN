#include <iostream>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int32MultiArray.h"

#include "tsp/point.h"
#include "tsp/line.h"
#include "tsp/rkga.h"
#include "tsp/util.h"

#define NUM_POINTS 50
Point points[NUM_POINTS];



void randomIntializePoints(){
    srand(time(NULL));
    for(Point & point: points){//need to use reference to change x and y
        point.x = randomFloat(0.0, 10.0);
        point.y = randomFloat(0.0, 10.0);
    }
    return;
}

std::vector<int> ga_main(Point points[NUM_POINTS]){
    RKGA ga = RKGA(0.35, 0.55, 0.30, 0.005, NUM_POINTS, 3000, 20);
    ga.initialize(points);
    int generation = 0;
    while(generation < ga.MaxGeneration){
        int selection_size = ga.select(points);
        int crossover_size = ga.crossover(points);
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
    randomIntializePoints();

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