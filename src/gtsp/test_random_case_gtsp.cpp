#include <iostream>
#include <time.h>    //reset random seed time(NULL)

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "tsp/GtspPath.h"

#include "point.h"
#include "gtsp/rkga.h"
#include "util.h"



std::vector<std::pair<int,int>> ga_main(RKGA & ga, std::vector<Set> &sets){
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
    ros::Publisher chatter_pub = n.advertise<tsp::GtspPath>("/path", 1000);
    ros::Rate loop_rate(1);
    srand(time(NULL));
    // to test GA first initialize points randomly
    std::vector<Set> sets = readSets();
    // RKGA ga = RKGA(points, 0.35, 0.55, 0.30, 0.005, 3000, 20);
    RKGA ga = RKGA(sets,0.35, 0.55, 0.30, 0.005, 1000, 10);
    ga.initialize();
    int count = 0;
    while(ros::ok()){
        tsp::GtspPath msg;
        std::vector<std::pair<int,int>> path = ga_main(ga, sets);
        std::vector<int> set_index, point_index;
        for(std::pair<int, int> & p: path){
            set_index.push_back(p.first);
            point_index.push_back(p.second);
        }
        msg.set_index = set_index;
        msg.point_index = point_index;
        chatter_pub.publish(msg);
        ros::spinOnce();
        std::cout<<count<<'\n';
        loop_rate.sleep();
        count ++;
    }
    return 0;
}