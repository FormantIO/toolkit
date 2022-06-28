
#include <ros_type_introspection/ros_introspection.hpp>
#include <ros/master.h>
#include <ros/ros.h>
#include <topic_tools/shape_shifter.h>
#include <iostream>
#include <set> 
#include <vector> 

#include "config.h"
#include "recorder.hpp"

class Adapter{

public:

    inline Adapter(){

        Config config;
        setup_topics(config); 

    }
    
    inline void run(){

        for(auto t : topics_to_subscribe){
            std::cout << "Attempting to subscribe to " << t.name << std::endl; 
            recorder.add_recorder(t.name);         
        }

        ros::spin(); 

    }

private:

    Config config; 

    ros::master::V_TopicInfo all_topics;            // All currently published topics.
    ros::master::V_TopicInfo topics_to_subscribe;   // The topics which we are to subscribe to. 

    Recorder recorder; 

    /**
     * @brief Setup all the topics so we know what to subscribe to.
     * 
     * @param c 
     */
    inline void setup_topics(const Config & c){
        ros::master::getTopics(all_topics); 
        
        if(c.get_subscribe_to_all()){
            ros::master::getTopics(topics_to_subscribe);
            return; 
        }

        std::set<std::string> sub_topics; 

        for(auto t : topics_to_subscribe){
           sub_topics.insert(t.name); 
        }

        for(auto t : all_topics){
            if(sub_topics.find(t.name) == sub_topics.end())
                continue;
            
            topics_to_subscribe.push_back(t); 
        }
    }

};