#include "ros/ros.h"
#include <ros/package.h>
#include "std_srvs/Empty.h"
#include "tsp/point.h"
#include "tsp/util.h"
#include <time.h>    //reset random seed time(NULL)
#include<fstream>

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
        srand(time(NULL));
        ROS_INFO("Initialize %d points", num_points);

        std::string cwd_path = ros::package::getPath("tsp");
        std::string points_file = cwd_path + "/config/com.txt";

        std::ofstream off;
        // ios::trunc means first clear the file,
        // if file exist open, if not make a new file
        off.open(points_file,std::ios::trunc); 
        for(int i = 0; i < num_points; ++ i){
            float x = randomFloat(0.0, 10.0);
            float y = randomFloat(0.0, 10.0);
            off << x << ' ' << y<<std::endl;
        }
        off.close();//关闭文件
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

