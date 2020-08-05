#include "ros/ros.h"
#include "std_srvs/Empty.h"
#include "tsp/point.h"
#include "tsp/util.h"
#include <time.h>    //reset random seed time(NULL)


bool randomIntializePoints(tsp::InitializePoints::Request &req, 
        tsp::InitializePoints::Response &res){
    num_points
    // srand(time(NULL));
    // for(Point & point: points){//need to use reference to change x and y
    //     point.x = randomFloat(0.0, 10.0);
    //     point.y = randomFloat(0.0, 10.0);
    // }
    ROS_INFO("Initialize points randomly");
    return true;
}

int main(int argc, char ** argv){
    ros::init(argc, argv, "initialize_points");
    ros::NodeHandle n;
    int num_points;
    if (n.getParam("NUM_POINTS", num_points))
    {
      ROS_INFO("Got param NUM_POINTS: %d", num_points);
    }
    else
    {
      ROS_ERROR("Failed to get param 'NUM_POINTS'");
    }
    ros::ServiceServer service = n.advertiseService("initialize_points_service", randomIntializePoints);
    ros::spin();
    return 0;
}