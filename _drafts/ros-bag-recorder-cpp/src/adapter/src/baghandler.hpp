
#include <string>
#include <rosbag/bag.h>
#include <ros/ros.h>

#include "outgoing.hpp"
#include "config.h"
#include "utils.h"

#ifndef BAGHANDLER_H
#define BAGHANDLER_H

class RosBag
{

public:
    inline RosBag() {}

    inline RosBag(std::string name) : name(name)
    {
        bag = new rosbag::Bag();
    }

    inline void open()
    {
        try{
            bag->open(name + std::string(".active"), rosbag::bagmode::Write);
            is_open = true;
        }catch(rosbag::BagIOException){
            std::cout << "Error opening bag at "<< name + std::string(".active") << std::endl;
        }
    }

    inline void close()
    {
        is_open = false; 
        bag->close();
        delete bag;
        rename();
    }

    inline void write(const OutgoingMessage &outgoing)
    {
        if(!is_open){
            std::cout << "Error: bag is not open: " << name << std::endl; 
            return; 
        }
        bag->write(outgoing.get_topic(), outgoing.get_time(), outgoing.get_msg()); 
    }

    inline void operator=(RosBag &&rhs)
    {
        name = rhs.name;
        is_open = rhs.is_open; 
        bag = rhs.bag;
        rhs.bag = bag;
    }

    inline void operator=(RosBag &rhs)
    {
        name = rhs.name;
        is_open = rhs.is_open; 
        bag = rhs.bag;
    }

private:
    inline void rename()
    {
        std::rename((name + std::string(".active")).data(), name.data());
    }

    std::string name;
    rosbag::Bag *bag;
    bool is_open = false; 
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
        bag2_start_time = bag1_start_time + ros::Duration(bag_length - bag_overlap);
        bag2_end_time = bag2_start_time + ros::Duration(2 * bag_length - bag_overlap);
    }

    inline ~BagHandler(){
        bag1.close();
        bag2.close(); 
    }

    /**
     * @brief Write the outgoing message to the current open rosbags.
     *
     * @param outgoing
     */
    inline void write(const OutgoingMessage &outgoing)
    {
      
        check_bags(outgoing.get_time());

        bag1.write(outgoing);
        if (bag2_start_time < outgoing.get_time())
        {
            bag2.write(outgoing);
        }
    }

    inline void check_bags(ros::Time time)
    {

        if (time > bag1_end_time)
        {
            bag1.close();

            // Case 1: timestamp is still in range of bag2.
            //         In that case, we simply set bag1 as bag2 and then
            //         generate a new bag2 with the next time interval
            if (bag2_end_time > time)
            {
                bag1 = bag2;
                bag1_start_time = bag2_start_time;
                bag1_end_time = bag2_end_time;

                bag2 = RosBag(generate_bag_name());
                bag2.open();
                bag2_start_time = bag1_end_time - ros::Duration(bag_overlap);
                bag2_end_time = bag2_start_time + ros::Duration(bag_length);

                return;
            }

            // Case 2: timestamp is not in the range of bag1 or bag2,
            //         and we need to generate 2 brand new bags to use.
            // TODO: reduce some of the code duplication here.
            bag2.close();

            bag1 = RosBag(generate_bag_name());
            bag1.open();
            bag1_start_time = ros::Time::now();
            bag1_end_time = bag1_start_time + ros::Duration(bag_length);

            bag2 = RosBag(generate_bag_name());
            bag2.open(); 
            bag2_start_time = bag1_end_time - ros::Duration(bag_overlap);
            bag2_end_time = bag2_start_time + ros::Duration(bag_length);

            return;
        }
    }

private:
    inline std::string generate_bag_name()
    {
        std::string bag_naming_convention = config.get_bag_naming_convention();
        adapter_utils::replaceAll(bag_naming_convention, "$bn", std::to_string(bag_index));
        adapter_utils::replaceAll( bag_naming_convention, "$dt", adapter_utils::get_time(config.get_date_time_string()) );
        bag_index += 1;
        std::string storage_path = config.get_bag_storage_path();
        return storage_path + std::string("/") + bag_naming_convention;
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