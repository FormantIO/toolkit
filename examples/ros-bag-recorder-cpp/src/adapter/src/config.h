/**
 * @file config.h
 *
 * @brief This file houses the config allows grabs configuration
 *        parameters from various locations.
 * @version 0.1
 * @date 2022-06-29
 *
 * @copyright Copyright (c) 2022
 *
 */

#include <fstream>
#include <string>
#include <vector>
#include <cstdlib>

#include <grpcpp/grpcpp.h>
#include <nlohmann/json.hpp> 

#include "rsjp.hpp"
#include "client.h"

#ifndef CONFIG_H
#define CONFIG_H

using json = nlohmann::json;

class Config
{
public:
    inline Config()
    {
        
        FormantAgentClient fclient; 

        std::string blob_data(fclient.get_blob_data()); 
        std::ifstream in_f2("config.json");

        try
        {
            json blob_json = json::parse(blob_data); 

            if(blob_json.contains("bag_recorder_config"))
                 blob_config = blob_json["bag_recorder_config"]; 
            else
                blob_config = json(); 
        }
        catch (nlohmann::json::parse_error)
        {
            blob_config = json();
        }

        try
        {
            in_f2 >> file_config;
        }
        catch (nlohmann::json::parse_error)
        {
            file_config = json();
        }

        configurations = {&blob_config, &file_config};

        load_config();
    }

    /**
     * @brief Get the subscribe_to_all configuration param
     *
     * @return true
     * @return false
     */
    bool get_subscribe_to_all() const
    {
        return subscribe_to_all.resource;
    }

    /**
     * @brief Get a vector of the specified topics to subscribe to.
     *
     * @return true
     * @return false
     */
    std::vector<std::string> get_topics() const
    {
        return topics.resource;
    }

    /**
     * @brief Get a list of topics which are to be ignored by the bag recorder
     *
     * @return std::vector<std::string>
     */
    std::vector<std::string> get_ignore_topics() const
    {
        return ignore_topics.resource;
    }

    /**
     * @brief Get the topic refresh rate for checking for new topics
     *
     * @return int
     */
    int get_topic_refresh_rate() const
    {
        return topic_refresh_rate.resource;
    }

    /**
     * @brief Get the bag length configuration param
     *
     * @return double
     */
    int get_bag_length() const
    {
        return bag_length.resource;
    }

    /**
     * @brief Get the bag_overlap configuration param
     *
     * @return double
     */
    int get_bag_overlap() const
    {
        return bag_overlap.resource;
    }

    /**
     * @brief Get the bag_storage_path configuration param
     *
     * @return std::string
     */
    std::string get_bag_storage_path() const
    {
        return bag_storage_path.resource;
    }

    /**
     * @brief Get the bag_naming_convention configuration param
     *
     * @return std::string
     */
    std::string get_bag_naming_convention() const
    {
        return bag_naming_convention.resource;
    }

    /**
     * @brief Get the date_time_string format
     *
     * @return std::string
     */
    std::string get_date_time_string() const
    {
        return date_time_string.resource;
    }

private:
    /**
     * @brief Load all the configuration parameters into memory
     *
     */
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
        subscribe_to_all.set_resource(resource_loader("subscribe_to_all", true, boolean));
    }

    void load_topics()
    {

        std::vector<std::string> _topics;
        for (auto c : configurations)
        {
            if (!type_check_param(c, "topics", string, true))
                continue;

            for (int i = 0; i < (*c)["topics"].size(); ++i)
            {
                _topics.push_back((*c)["topics"][i]);
            }

            topics.set_resource(_topics);
            return;
        }
        topics.set_resource(_topics);
    }

    void load_ignore_topics()
    {
        // TODO: Not implemented yet
    }

    void load_topic_refresh_rate()
    {
        topic_refresh_rate.set_resource(resource_loader<double>("topic_refresh_rate", 2, number));
    }

    void load_bag_length()
    {
        bag_length.set_resource(resource_loader<double>("bag_length", 5, number));
    }

    void load_bag_overlap()
    {
        bag_overlap.set_resource(resource_loader<double>("bag_overlap", 2, number));
    }

    void load_bag_storage_path()
    {
        bag_storage_path.set_resource(resource_loader<std::string>("bag_storage_path", "./", string));
    }

    void load_bag_naming_convention()
    {
        bag_naming_convention.set_resource(resource_loader<std::string>("bag_naming_convention", "demo-bag$bn-$dt.bag", string));
    }

    void load_date_time_string()
    {
        date_time_string.set_resource(resource_loader<std::string>("date_time_string", "%d_%m_%Y-%H_%M_%S", string));
    }

    /**
     * @brief A config resource is a wrapper that holds
     *        configuration parameter along with
     *        telling us if the resource has loaded.
     *
     * @tparam T
     */
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

    json blob_config;
    json file_config;

    std::vector<json *> configurations;

    enum config_types
    {
        string,
        number,
        boolean
    };

    template <typename T>
    inline T resource_loader(const std::string &param, T def, config_types type)
    {

        for (auto config : configurations)
        {

            const auto &c(*config);

            if (!c.contains(param))
                continue;

            if (c[param].is_null())
                continue;

            if (!type_check_param(config, param, type, false))
                continue;

            return c[param];
        }
        return def;
    }

    inline bool type_check_param(
        const json *config,
        const std::string &param,
        const config_types type,
        const bool is_array)
    {
        const json &cur_config = *config;

        if (!config->contains(param))
            return false;

        if (!is_array)
        {
            return type_check_value(cur_config[param], type);
        }
        else
        {

            if (!cur_config[param].is_array())
                return false;

            for (int i = 0; i < cur_config[param].size(); ++i)
            {
                if (!type_check_value(cur_config[param][i], type))
                    return false;
            }

            return true;
        }
    }

    inline bool type_check_value(
        const nlohmann::json &value,
        const config_types type)
    {

        if (value.is_null())
            return false;

        switch (type)
        {
        case config_types::boolean:
            return value.is_boolean();
        case config_types::number:
            return value.is_number();
        case config_types::string:
            return value.is_string();
        default:
            return false;
        }
    }

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