#ifndef TSP_UTIL_H
#define TSP_UTIL_H

#include <assert.h>
#include <stdlib.h>     /* srand, rand */
#include "tsp/point.h"
#include <sstream>
#include <ros/package.h>
#include<fstream>

static std::vector<Point> readPoints(){
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

inline float randomFloat(float min, float max)
{
    // this  function assumes max > min, you may want 
    // more robust error checking for a non-debug build
    assert(max > min); 
    float random = ((float) rand()) / (float) RAND_MAX;

    // generate (in your case) a float between 0 and (4.5-.78)
    // then add .78, giving you a float between .78 and 4.5
    float range = max - min;  
    return (random*range) + min;
}

// generate random int [min. max]
inline int randomInt(int min, int max)
{
    // this  function assumes max > min, you may want 
    // more robust error checking for a non-debug build
    assert(max > min); 
    return min + rand() % (max - min + 1);
}

#endif