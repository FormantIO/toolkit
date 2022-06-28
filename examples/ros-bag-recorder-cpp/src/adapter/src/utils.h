#include <ros/master.h>

#ifndef UTILS_H
#define UTILS_H

ros::master::V_TopicInfo get_topics()
{

    ros::master::V_TopicInfo master_topics;
    ros::master::getTopics(master_topics);
    return master_topics;    

}
#endif