
#include <string>
#include <rosbag/bag.h>
#include <ros/ros.h>

#include "outgoing.hpp"
#include "config.h"
#include "utils.h"

#ifndef BAGHANDLER_H
#define BAGHANDLER_H

class RosBag{

public:

    inline RosBag() {} 

    inline RosBag(std::string name) : name(name){
        bag = new rosbag::Bag(); 
    }

    inline void open(){
        bag->open(name, rosbag::bagmode::Write); 
    }

    inline void close(){
        bag->close(); 
        // delete bag; 
    }

    inline void write(const OutgoingMessage & outgoing){
        bag->write(outgoing.get_topic(), outgoing.get_time(), outgoing.get_msg());
    }

    inline void operator=(RosBag && rhs){
        name = rhs.name;
        bag = rhs.bag; 
        rhs.bag = bag; 
    }

    inline void operator=(RosBag & rhs){
        name = rhs.name; 
        bag = rhs.bag; 
    }

private:

    inline void rename(){

    }

    std::string name;
    rosbag::Bag * bag; 

};

class BagHandler
{

public:
    inline BagHandler()
    {
        bag_length = config.get_bag_length();
        bag_overlap = config.get_bag_overlap();

        bag1 = RosBag(generate_bag_name()); 
        bag1.open(); 
        bag1_start_time = ros::Time::now(); 
        bag1_end_time = bag1_start_time + ros::Duration(bag_length);

        bag2 = RosBag(generate_bag_name()); 
        bag2.open(); 
        bag1_start_time = ros::Time::now(); 
        bag1_end_time = bag1_start_time + ros::Duration(bag_length / 2);
    }

    inline ~BagHandler()
    {
        bag1.close();
    }

    /**
     * @brief Write the outgoing message to the current open rosbag's.
     *
     * @param outgoing
     */
    inline void write(const OutgoingMessage &outgoing)
    {
        std::cout << "Writing to bag" << std::endl;
     
        check_bags( outgoing.get_time() ); 
        
        bag1.write(outgoing);
        if(bag2_start_time < outgoing.get_time()){
            bag2.write(outgoing); 
        }
        

    }

    inline void check_bags(ros::Time time){

        if(time > bag1_end_time){
            bag1.close(); 


            // Case 1: timestamp is still in range of bag2.
            //         In that case, we simply set bag1 as bag2 and then
            //         generate a new bag2 with the next time interval
            if(bag2_end_time > time){
                bag1 = bag2; 
                bag1_start_time = bag2_start_time; 
                bag1_end_time = bag2_end_time; 

                bag2 = RosBag(generate_bag_name()); 
                bag2.open(); 
                bag2_start_time = bag1_end_time - ros::Duration(bag_overlap); 
                bag2_end_time = bag2_start_time + ros::Duration(bag_length); 

                return; 

            }

            // Case 2: neither bag1 nor bag2 contain the timestamp. 
            //       : In this case, we need to jump up to the proper
            //       : interval in the bagging timeline.
            //       : The following does that. 

            std::cout << "Error: Case 2 Not implemented. Exiting" << std::endl;
            bag1.close();
            bag2.close(); 
            exit(1); 

        }

    }

private:

    inline std::string generate_bag_name(){
        std::string bag_naming_convention = "bag$bn.bag";
        bag_naming_convention += ".active"; 
        adapter_utils::replaceAll(bag_naming_convention, "$bn", std::to_string(bag_index)); 
        return bag_naming_convention; 
    }

    Config config;
    std::string bag_save_path;
    int bag_index = 0;

    RosBag bag1;
    ros::Time bag1_start_time;
    ros::Time bag1_end_time; 

    RosBag bag2; 
    ros::Time bag2_start_time;
    ros::Time bag2_end_time; 

    int bag_length;
    int bag_overlap; 

};



#endif