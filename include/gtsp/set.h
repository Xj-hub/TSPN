#ifndef GTSP_SET_H
#define GTSP_SET_H

#include "tsp/point.h"
#include <vector>

struct Set{
    int index;
    std::vector<Point> points;
    Set():index(-1), points(std::vector<Point>()){};
    Set(int index, std::vector<Point> & points):index(index), points(points){};
};

#endif