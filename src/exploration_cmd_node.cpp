#include <ros/ros.h>
#include "exploration_cmd.h"
#include "exploration_cmd.cpp"

using namespace std;

int main(int argc, char** argv) {
    ros::init(argc, argv, "Exploration_Command_Node");
    const ros::NodeHandle& private_nh = ros::NodeHandle("~");

    ExplorationCMD explorationCmd;
    ros::spinOnce();
    
    try{
        ros::spin();
    }catch(std::runtime_error& e){
        ROS_ERROR("octomap_server exception: %s", e.what());
        return -1;
    }

    return 0;
}