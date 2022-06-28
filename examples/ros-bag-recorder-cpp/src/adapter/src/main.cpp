

#include <iostream>
#include <ros/ros.h>

#include "adapter.h" 


int main(int argc, char **argv){

    std::cout << "Starting adapter" << std::endl; 

    ros::init(argc, argv, "FormantRosbagAdapter");  
    Adapter adapter; 
    adapter.run(); 
}