#ifndef DISTANCE_H
#define DISTANCE_H

#include "point.h"
#include<math.h>
float euclideanDistance(Point p1, Point p2){
    float dist;
    float x = p1.x - p2.x; //calculating number to square in next step
	double y = p1.y - p2.y;
	dist = pow(x, 2) + pow(y, 2);       //calculating Euclidean distance
	dist = sqrt(dist);                  

	return dist;
}

#endif