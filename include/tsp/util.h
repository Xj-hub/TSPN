#ifndef TSP_UTIL_H
#define TSP_UTIL_H

#include <assert.h>
#include <stdlib.h>     /* srand, rand */
#include "tsp/point.h"


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