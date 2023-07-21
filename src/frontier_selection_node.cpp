#include <ros/ros.h>
#include "frontier_selection.h"
#include "frontier_selection.cpp"

using namespace std;

int main(int argc, char** argv) {
    ros::init(argc, argv, "Frontier_Exploration_Nodes");
    const ros::NodeHandle& private_nh = ros::NodeHandle("~");

    FrontierSelection frontierSelection;
    ros::spinOnce();
    
    
    try{
        ros::spin();
    }catch(std::runtime_error& e){
        ROS_ERROR("octomap_server exception: %s", e.what());
        return -1;
    }

    return 0;
}