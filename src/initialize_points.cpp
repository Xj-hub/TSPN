#include "ros/ros.h"
#include "std_srvs/Empty.h"
#include "tsp/point.h"
#include "tsp/util.h"
#include <time.h>    //reset random seed time(NULL)

class PointInitializer{
private:
    int num_points;
    ros::ServiceServer initial_points_service;

public:
    PointInitializer(ros::NodeHandle *nh){
        if (nh->getParam("NUM_POINTS", num_points))
        {
        ROS_INFO("Got param NUM_POINTS: %d", num_points);
        }
        else
        {
        ROS_ERROR("Failed to get param 'NUM_POINTS'");
        }
        initial_points_service = nh->advertiseService("initialize_points_service", &PointInitializer::callback, this);
    }

    bool callback(std_srvs::Empty::Request &req,
            std_srvs::Empty::Response &res){
        
        ROS_INFO("Initialize points randomly");
        return true;
    }
};

int main(int argc, char ** argv){
    ros::init(argc, argv, "initialize_points");
    ros::NodeHandle nh;
    PointInitializer pointInitializer = PointInitializer(&nh);
    ros::spin();
    return 0;
}

