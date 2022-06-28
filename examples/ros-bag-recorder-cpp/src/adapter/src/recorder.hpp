

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

    inline void topicCallback(const topic_tools::ShapeShifter::ConstPtr &msg,
                       const std::string &topic_name)
    {
        boost::mutex::scoped_lock lock(queue_mutex);  
        std::cout << "New Message on " << topic_name << "\n"; 

        OutgoingMessage outgoing(topic_name, msg, ros::Time::now()); 
        
        handler.write(outgoing);
    }

private:
    std::vector<ros::Subscriber> current_subscribers;
    std::set<std::string> current_topics;

    boost::mutex queue_mutex;
    std::queue<OutgoingMessage> message_queue;

    BagHandler handler; 

};

#endif