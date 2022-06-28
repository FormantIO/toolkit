
#include "outgoing.hpp"
#include "config.h"
#include "rosbag/bag.h"

#ifndef BAGHANDLER_H
#define BAGHANDLER_H

class BagHandler{

public: 

    inline BagHandler(){
        bag1.open("test.bag", rosbag::bagmode::Write); 
    }

    inline ~BagHandler(){
        bag1.close(); 
    }

    /**
     * @brief Write the outgoing message to the current open rosbag's.
     * 
     * @param outgoing 
     */
    inline void write(const OutgoingMessage& outgoing){
        std::cout << "Writing to bag" << std::endl;
        bag1.write(outgoing.get_topic(), outgoing.get_time(), outgoing.get_msg()); 
    }

private:

Config config; 

rosbag::Bag bag1;

};

#endif