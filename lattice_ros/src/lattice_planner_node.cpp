#include <ros/ros.h>
#include "lattice_planner/lattice_planner.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "lattice_planner_node");
    
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    
    try {
        lattice_planner::LatticePlanner planner(nh, pnh);
        
        if (!planner.initialize()) {
            ROS_ERROR("Failed to initialize lattice planner");
            return -1;
        }
        
        ROS_INFO("Lattice planner node started successfully");
        planner.run();
        
    } catch (const std::exception& e) {
        ROS_ERROR("Exception in lattice planner: %s", e.what());
        return -1;
    }
    
    return 0;
}