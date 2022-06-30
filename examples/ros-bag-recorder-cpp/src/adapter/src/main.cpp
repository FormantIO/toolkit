/**
 * @file main.cpp
 * @brief This file boots the adapter for the user. 
 * @version 0.1
 * @date 2022-06-29
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include <iostream>
#include <ros/ros.h>

#include "adapter.h" 


int main(int argc, char **argv){

    // Setup ROS
    ros::init(argc, argv, "FormantRosbagAdapter");  
    ros::start();
    
    // Start running the adapter
    Adapter adapter; 
    adapter.run();
    
    // Shutdown the adapter (This does not currently run).
    ros::shutdown();  
}