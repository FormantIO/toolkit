#include <ros/master.h>
#include <ctime>
#include <sstream>
#include <iostream>
#include <sstream>
#include <ctime> 
#include <iomanip> 

#ifndef UTILS_H
#define UTILS_H

namespace adapter_utils
{

    inline ros::master::V_TopicInfo get_topics()
    {

        ros::master::V_TopicInfo master_topics;
        ros::master::getTopics(master_topics);
        return master_topics;
    }

    inline std::string get_time(const std::string &time_fmt)
    {
        auto t = std::time(nullptr);
        auto tm = *std::localtime(&t);
        std::stringstream ss;
        ss << std::put_time(&tm, time_fmt.c_str());
        return ss.str();
    }

    inline bool replace(std::string &str, const std::string &from, const std::string &to)
    {
        size_t start_pos = str.find(from);
        if (start_pos == std::string::npos)
            return false;
        str.replace(start_pos, from.length(), to);
        return true;
    }

    inline void replaceAll(std::string &str, const std::string &from, const std::string &to)
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