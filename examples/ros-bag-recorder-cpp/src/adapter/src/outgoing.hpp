#include <topic_tools/shape_shifter.h>
#include <boost/thread/condition.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/regex.hpp>

#ifndef OUTGOING_H
#define OUTGOING_H

class OutgoingMessage
{
public:
    inline OutgoingMessage(std::string const &topic, topic_tools::ShapeShifter::ConstPtr msg, ros::Time time)
    :  topic(topic), msg(msg), time(time)
    {
    }

    inline std::string get_topic() const{
        return topic; 
    }

    inline auto get_msg() const{
        return msg;
    }

    inline ros::Time get_time() const{
        return time; 
    }

private:

    std::string topic;
    topic_tools::ShapeShifter::ConstPtr msg;
    ros::Time time; 

};

#endif