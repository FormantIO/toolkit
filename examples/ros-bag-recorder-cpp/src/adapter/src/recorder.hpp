/**
 * @file recorder.hpp
 * @brief This file houses the recorder which sets up ROS subscribers
 * 
 * @version 0.1
 * @date 2022-06-29
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include <ros_type_introspection/ros_introspection.hpp>
#include <ros/master.h>
#include <ros/ros.h>
#include <topic_tools/shape_shifter.h>
#include <iostream>
#include <vector>
#include <set>
#include <queue>
#include <boost/thread/mutex.hpp>

#include "baghandler.hpp"
#include "outgoing.hpp"

#ifndef RECORDER_H
#define RECORDER_H

class Recorder
{

public:

    /**
     * @brief Add a new message recorder which will record messages to a bag. 
     * 
     * @param topic 
     */
    inline void add_recorder(const std::string &topic)
    {

        if (current_topics.find(topic) != current_topics.end())
            return; // Already subscribed

        ros::NodeHandle nh; 

        boost::function<void(const topic_tools::ShapeShifter::ConstPtr&msg)> callback;
        callback = [this, topic] (const topic_tools::ShapeShifter::ConstPtr & msg){
            this->topicCallback(msg, topic); 
        }; 

        ros::Subscriber sub = nh.subscribe(topic, 10, callback); 

        current_subscribers.push_back(sub); 
        current_topics.insert(topic); 
    }

    /**
     * @brief ROS subscriber callback where incoming messages are sent to.
     * 
     * @param msg 
     * @param topic_name 
     */
    inline void topicCallback(const topic_tools::ShapeShifter::ConstPtr &msg,
                       const std::string &topic_name)
    {
        // Get the mutex
        boost::mutex::scoped_lock lock(queue_mutex);  
        OutgoingMessage outgoing(topic_name, msg, ros::Time::now());

        // We can safely write to the bag as this will be the only instance 
        // with the mutex. This forms a sort of 'queue' by only allowing the 
        // thread with the mutex to write. 
        handler.write(outgoing); 
    }

private:
    std::vector<ros::Subscriber> current_subscribers;
    std::set<std::string> current_topics;

    boost::mutex queue_mutex;

    BagHandler handler; 

};

#endif