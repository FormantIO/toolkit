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
#include <thread>
#include <set>

#include "client.h"
#include "baghandler.hpp"
#include "outgoing.hpp"
#include "config.h"

#ifndef RECORDER_H
#define RECORDER_H

class Recorder
{

public:
    /**
     * @brief Construct a new recorder object
     *
     */
    inline Recorder()
    {
        std::vector<std::string> topic_filter = {
            "start_ros_recorder",
            "stop_ros_recorder",
            "start_ros_recorder_duration"};

        auto cb = [this](const GetCommandRequestStreamResponse &msg)
        {
            this->handle_command(msg);
        };

        FormantAgentClient::FormantAgentCallback callback(cb);

        fclient.register_command_callback(callback, topic_filter);

        refresh_thread = std::thread(&Recorder::refresh_thread_target, this);
    }

    ~Recorder()
    {
        recorder_shutdown = true;
        refresh_thread.join();
    }

    /**
     * @brief Add a new message recorder which will record messages to a bag.
     *
     * @param topic - The topic to add a recorder to
     * @param no_subscription - force the call to not add a subscriber to specified topic on this call
     */
    inline void add_recorder(const std::string &topic, bool no_subscription = false)
    {
        {
            boost::mutex::scoped_lock lock(current_topics_mutex);
            boost::mutex::scoped_lock lock_new_subs(create_subscriber_mutex);

            if (current_topics.find(topic) != current_topics.end())
                return; // Already subscribed
            create_subscriber.insert(topic);
        }

        if (is_recording && !no_subscription)
            subscribe_to_topics();
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

    /**
     * @brief Start the ROS recorder by subscribing to the topics
     *
     */
    inline void start_recording()
    {
        if (is_recording)
            return;
        is_recording = true;
        refresh_topics();
        subscribe_to_topics();
    }

    /**
     * @brief Subscribe to all topics in this->current_topics
     *
     */
    inline void subscribe_to_topics()
    {

        boost::mutex::scoped_lock current_topics_lock(current_topics_mutex);
        boost::mutex::scoped_lock current_subscribers_lock(current_subscribers_mutex);
        boost::mutex::scoped_lock create_subscriber_lock(create_subscriber_mutex); 

        for (auto &topic : create_subscriber)
        {

            // We've already created the subscriber
            if (current_topics.count(topic))
                continue;

            ros::NodeHandle nh;

            boost::function<void(const topic_tools::ShapeShifter::ConstPtr &msg)> callback;
            callback = [this, topic](const topic_tools::ShapeShifter::ConstPtr &msg)
            {
                this->topicCallback(msg, topic);
            };

            ros::Subscriber sub = nh.subscribe(topic, 10, callback);

            current_subscribers.push_back(sub);
            current_topics.insert(topic);

        }

        create_subscriber.clear(); 
    }

    /**
     * @brief Stop the ROS recorder from recording
     *
     */
    inline void stop_recording()
    {
        boost::mutex::scoped_lock lock(current_subscribers_mutex);
        current_subscribers.clear();
        current_topics.clear();
        is_recording = false;
    }

    /**
     * @brief Ask the recorder to record ROS messages for a specified duration of time
     *
     * @param duration
     */
    inline void record_duration(ros::Duration duration)
    {
        start_recording();
        sleep(duration.toSec());
        stop_recording();
    }

private:
    /**
     * @brief The callback which is used internally when a new command is received from the agent.
     *
     * @param msg
     */
    inline void handle_command(const GetCommandRequestStreamResponse &msg)
    {
        std::string command = msg.request().command();
        std::string payload = msg.request().text();
        if (command == "start_ros_recorder_duration")
        {
            double command_length = std::stod(payload);
            ros::Duration duration(command_length);
            record_duration(duration);
            return;
        }

        else if (command == "start_ros_recorder")
        {
            start_recording();
            return;
        }

        else if (command == "stop_ros_recorder")
        {
            stop_recording();
            return;
        }
    }

    /**
     * @brief Refresh the current topics and add new ones to the recorder
     *
     */
    inline void refresh_topics(bool purge = false)
    {
        boost::mutex::scoped_lock lock(refresh_mutex);
        ros::master::getTopics(all_topics);

        // If we need to subscribe to all the topics, then
        // simply setup attempt to subscribe to each one and the
        // recorder will take care of what we are already subscribed to.
        if (config.get_subscribe_to_all())
        {

            for (size_t i = 0; i < all_topics.size(); ++i)
            {
                add_recorder(all_topics[i].name, i != all_topics.size() - 1);
            }

        }

        // In the case that we don't want to subscribe to
        // everything, we go through the configured topics
        // and attempt to subscribe to them.

        // Step 1: create a set from the published topics
        std::set<std::string> available_topics;
        for (auto &t : all_topics)
        {
            available_topics.insert(t.name);
        }

        // Step 2: purge if requested
        if(purge){
            
            boost::mutex::scoped_lock current_topics_lock(current_topics_mutex); 

            // This first gathers the topics which we are to purge. 
            std::set<std::string> purge_topics; 
            for(auto & t : current_topics){
                if(!available_topics.count(t)){
                    purge_topics.insert(t);  
                }
            }

            // Now we are going to actually remove the purged topics
            if(purge_topics.size()){
                boost::mutex::scoped_lock sub_lock(current_subscribers_mutex); 
                std::vector<ros::Subscriber> current_subscribers_new;  
                std::set<std::string> current_subscription_topics_new; 

                for(auto & s: current_subscribers){
                    // s.getTopic()

                    if(purge_topics.count(s.getTopic()))
                        continue; 
                    
                    current_subscribers_new.push_back(s); 
                    current_subscription_topics_new.insert(s.getTopic()); 

                }

                current_subscribers = current_subscribers_new; 
                current_topics = current_subscription_topics_new; 

            }

        }

        if(config.get_subscribe_to_all())
            return; 

        // Step 3: Add a recorder for topics
        auto add_topics = config.get_topics();
        for (int i = 0; i < add_topics.size(); ++i)
        {
            if (available_topics.count(add_topics[i]))
                add_recorder(add_topics[i], i != add_topics.size() - 1);
        }

        
    }

    /**
     * @brief Target for the refresh thread. This will refresh topics at the specified 
     *        refresh rate while also listen for a shutdown command in a non-blocking 
     *        manner. 
     * 
     */
    inline void refresh_thread_target()
    {
        ros::Time time = ros::Time::now();
        ros::Time last_update = ros::Time::now();
        double loop_refresh_rate =
            config.get_topic_refresh_rate() < 1 ? config.get_topic_refresh_rate() : 1;
        double topic_refresh_rate = config.get_topic_refresh_rate(); 
        while (ros::ok() && !recorder_shutdown)
        {
            sleep(loop_refresh_rate);
            if (is_recording && ros::Time::now() - last_update > ros::Duration(topic_refresh_rate))
            { 
                refresh_topics(true);
                last_update = ros::Time::now();
            }
        }
    }


    Config config; // Load configuration parameters.

    boost::mutex refresh_mutex; // Only one thread can refresh the topics at a time.
    std::thread refresh_thread; // The thread that is used to refresh topics.

    std::vector<ros::Subscriber> current_subscribers;  // All the current ROS subscribers
    boost::mutex current_subscribers_mutex;            // Mutex to access the data structure

    std::set<std::string> create_subscriber; // Topics which we need to create new subscribers for
    boost::mutex create_subscriber_mutex; // Mutex for above data structure 

    std::set<std::string> current_topics; // The topics which are currently being subscribed to
    boost::mutex current_topics_mutex;    // mutex to access the current topics data structure

    boost::mutex queue_mutex; // The mutex which is locked when writing to the BagHandler

    ros::master::V_TopicInfo all_topics; // All currently published topics.

    BagHandler handler; // The tool which is used to write directly to a ROS bag

    FormantAgentClient fclient; // The Formant Client

    bool is_recording = false; // Is the adapter currently recording?

    bool recorder_shutdown = false; // Should loops and threads return?
};

#endif