
#include <fstream>
#include "rsjp.hpp"
#include <string>
#include <vector>

#ifndef CONFIG_H
#define CONFIG_H

class Config
{

public:

    inline Config()
    {
        std::ifstream infile("config.json");
        RSJresource c(infile);
        config = c;
        load_config();
    }

    bool get_subscribe_to_all()
    {
        return subscribe_to_all.resource;
    }

    std::vector<std::string> get_topics()
    {
        return topics.resource;
    }

    std::vector<std::string> get_ignore_topics()
    {
        return ignore_topics.resource;
    }

    int get_topic_refresh_rate()
    {
        return topic_refresh_rate.resource;
    }

    int get_bag_length()
    {
        return bag_length.resource;
    }

    int get_bag_overlap()
    {
        return bag_overlap.resource;
    }

    std::string get_bag_storage_path()
    {
        return bag_storage_path.resource;
    }

    std::string get_bag_naming_convention()
    {
        return bag_naming_convention.resource;
    }

    std::string get_date_time_string()
    {
        return date_time_string.resource;
    }

private:
    template <class T>
    inline T resource_loader(std::string param, T default_val)
    {
        return config[param].as<T>(default_val);
    }

    void load_config()
    {
        load_subscribe_to_all();
        load_topics();
        load_ignore_topics();
        load_topic_refresh_rate();
        load_bag_length();
        load_bag_overlap();
        load_bag_storage_path();
        load_bag_naming_convention();
        load_date_time_string();
    }

    void load_subscribe_to_all()
    {
        subscribe_to_all.set_resource(resource_loader("subscribe_to_all", true));
    }

    void load_topics()
    {
        std::vector<std::string> _topics;
        for (auto topic : config["topics"].as_array())
        {
            _topics.push_back(topic.as<std::string>());
        }
        topics.set_resource(_topics);
    }

    void load_ignore_topics()
    {
        std::vector<std::string> _ignore_topics;
        for (auto topic : config["ignore_topics"].as_array())
        {
            _ignore_topics.push_back(topic.as<std::string>());
        }
        ignore_topics.set_resource(_ignore_topics);
    }

    void load_topic_refresh_rate()
    {
        topic_refresh_rate.set_resource(resource_loader<double>("topic_refresh_rate", 2));
    }

    void load_bag_length()
    {
        bag_length.set_resource(resource_loader<double>("bag_length", 2));
    }

    void load_bag_overlap()
    {
        bag_overlap.set_resource(resource_loader<double>("bag_overlap", 2));
    }

    void load_bag_storage_path()
    {
        bag_storage_path.set_resource(resource_loader<std::string>("bag_overlap", "/home/fvolcic/bags"));
    }

    void load_bag_naming_convention()
    {
        bag_naming_convention.set_resource(resource_loader<std::string>("bag_storage_path", "demo-bag$bn-$dt.bag"));
    }

    void load_date_time_string()
    {
        date_time_string.set_resource(resource_loader<std::string>("bag_storage_path", "demo-bag$bn-$dt.bag"));
    }

    template <typename T>
    struct ConfigResource
    {

        inline ConfigResource() {}
        inline ConfigResource(T resource) : resource(resource) {}

        inline void set_resource(T value)
        {
            resource = value;
            loaded = true;
        }

        T resource;
        bool loaded = false;
    };

    RSJresource config;

    ConfigResource<bool> subscribe_to_all;
    ConfigResource<std::vector<std::string>> topics;
    ConfigResource<std::vector<std::string>> ignore_topics;
    ConfigResource<double> topic_refresh_rate;
    ConfigResource<double> bag_length;
    ConfigResource<double> bag_overlap;
    ConfigResource<std::string> bag_storage_path;
    ConfigResource<std::string> bag_naming_convention;
    ConfigResource<std::string> date_time_string;
};

#endif