#include <ros/master.h>
#include <ctime>
#include <sstream>

#ifndef UTILS_H
#define UTILS_H

namespace adapter_utils
{

    ros::master::V_TopicInfo get_topics()
    {

        ros::master::V_TopicInfo master_topics;
        ros::master::getTopics(master_topics);
        return master_topics;
    }

    bool replace(std::string &str, const std::string &from, const std::string &to)
    {
        size_t start_pos = str.find(from);
        if (start_pos == std::string::npos)
            return false;
        str.replace(start_pos, from.length(), to);
        return true;
    }

    void replaceAll(std::string &str, const std::string &from, const std::string &to)
    {
        if (from.empty())
            return;
        size_t start_pos = 0;
        while ((start_pos = str.find(from, start_pos)) != std::string::npos)
        {
            str.replace(start_pos, from.length(), to);
            start_pos += to.length(); // In case 'to' contains 'from', like replacing 'x' with 'yx'
        }
    }

   

}
#endif