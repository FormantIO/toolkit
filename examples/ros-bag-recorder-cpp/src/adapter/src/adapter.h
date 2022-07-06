
#include <ros_type_introspection/ros_introspection.hpp>
#include <ros/master.h>
#include <ros/ros.h>
#include <topic_tools/shape_shifter.h>
#include <iostream>
#include <set>
#include <vector>
#include <thread>

#include "config.h"
#include "recorder.hpp"
#include "client.h"

class Adapter
{

public:
    inline Adapter()
    {

        Config config;
        setup_topics(config);
    }

    /**
     * @brief Run the adapter. This function will not return until the node is killed
     *        using the command line rosnode tool.
     *
     */
    inline void run()
    {

        for (; ros::ok();)
        {
            for (auto t : topics_to_subscribe)
            {
                recorder.add_recorder(t.name);
            }

            if (!config.get_subscribe_to_all())
                break;
            ros::Duration(config.get_topic_refresh_rate()).sleep();
            ros::master::getTopics(topics_to_subscribe);
        }

        ros::spin();
    }

private:
    Config config; // The configuration variable to load the config.

    ros::master::V_TopicInfo all_topics;          // All currently published topics.
    ros::master::V_TopicInfo topics_to_subscribe; // The topics which we are to subscribe to.

    Recorder recorder; // The recorder object used to receive and save incoming ros messages

    bool is_shutdown = false;

    /**
     * @brief Setup all the topics so we know what to subscribe to.
     *
     * @param c
     */
    inline void setup_topics(const Config &c)
    {
        ros::master::getTopics(all_topics);

        if (c.get_subscribe_to_all())
        {
            ros::master::getTopics(topics_to_subscribe);
            return;
        }

        std::set<std::string> sub_topics;

        for (auto t : config.get_topics())
        {
            sub_topics.insert(t);
        }

        for (auto t : all_topics)
        {
            if (sub_topics.find(t.name) == sub_topics.end())
                continue;

            topics_to_subscribe.push_back(t);
        }
    }
};