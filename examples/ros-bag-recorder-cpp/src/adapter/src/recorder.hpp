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

#include "client.h"
#include "baghandler.hpp"
#include "outgoing.hpp"

#ifndef RECORDER_H
#define RECORDER_H

class Recorder
{

public:
    inline Recorder()
    {
        std::cout << "Recorder Started!" << std::endl;
        std::vector<std::string> topic_filter = {
            "start_ros_recorder",
            "stop_ros_recorder",
            "start_ros_recorder_duration",
            "test"};

        auto cb = [this](const GetCommandRequestStreamResponse &msg)
        {
            this->handle_command(msg);
        };

        FormantAgentClient::FormantAgentCallback callback(cb);

        fclient.register_command_callback(callback, topic_filter);
    }

    inline void handle_command(const GetCommandRequestStreamResponse &msg)
    {
        std::cout << "Command received" << std::endl;

        std::string command = msg.request().command();
        std::string payload = msg.request().text();

        std::cout << command << " | " << payload << std::endl;

        if(command == "start_ros_recorder_duration"){
            double command_length = std::stod(payload); 
            ros::Duration duration(command_length); 
            record_duration(duration); 
            return; 
        }

        if(command == "start_ros_recorder"){
            start_recording();
            return;
        }

        if(command == "stop_ros_recorder"){
            stop_recording(); 
            return; 
        }


    }

    /**
     * @brief Add a new message recorder which will record messages to a bag.
     *
     * @param topic
     */
    inline void add_recorder(const std::string &topic)
    {
        if (current_topics.find(topic) != current_topics.end())
            return; // Already subscribed
        std::cout << "Subscribing to " << topic << std::endl; 
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
        std::cout << "New Message!" << std::endl; 
        
        // Get the mutex
        boost::mutex::scoped_lock lock(queue_mutex);
        OutgoingMessage outgoing(topic_name, msg, ros::Time::now());


        // We can safely write to the bag as this will be the only instance
        // with the mutex. This forms a sort of 'queue' by only allowing the
        // thread with the mutex to write.
        handler.write(outgoing);
    }

    inline void start_recording()
    {
        std::cout << "Bag Recorder Started" << std::endl; 
        if(is_recording)
            return;
        is_recording = true;

        for (auto &topic : current_topics)
        {

            std::cout << "generating sub to " << topic << std::endl;  

            ros::NodeHandle nh;

            boost::function<void(const topic_tools::ShapeShifter::ConstPtr &msg)> callback;
            callback = [this, topic](const topic_tools::ShapeShifter::ConstPtr &msg)
            {
                this->topicCallback(msg, topic);
            };

            ros::Subscriber sub = nh.subscribe(topic, 10, callback);

            current_subscribers.push_back(sub);
        }
 
        std::cout << "Subscribed to all topics" << std::endl; 
    }

    inline void stop_recording()
    {
        std::cout << "Bag Recorder Stopped" << std::endl; 
        current_subscribers.clear();
        is_recording = false; 
    }

    inline void record_duration(ros::Duration duration)
    {
        start_recording();
        sleep(duration.toSec());
        stop_recording();
    }


private:

    std::vector<ros::Subscriber> current_subscribers;
    std::set<std::string> current_topics;

    boost::mutex queue_mutex;

    BagHandler handler;

    FormantAgentClient fclient;

    bool is_recording = false; 

};

#endif