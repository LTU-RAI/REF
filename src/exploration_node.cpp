#include <ros/ros.h>
#include "exploration.h"
#include "exploration.cpp"

using namespace std;

int main(int argc, char** argv) {
    ros::init(argc, argv, "Frontier_Exploration_Node");
    const ros::NodeHandle& private_nh = ros::NodeHandle("~");

    FrontierExploration frontierExploration;
    ros::spinOnce();
    
    try{
        ros::spin();
    }catch(std::runtime_error& e){
        ROS_ERROR("octomap_server exception: %s", e.what());
        return -1;
    }

    return 0;
}