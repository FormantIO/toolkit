#include "rsjp.hpp"

#include <fstream>
#include <iostream>
#include "config.h"

int main()
{

    std::ifstream ifs("config.json");
    RSJresource config(ifs);
    config.parse();
    // std::cout << config["topics"].as<std::string>("_INVALID") << std::endl;
    // std::cout << config["subscribe_to_all"].as<std::string>("_INVALID") << std::endl;
    Config c;

    // std::cout << config["bag_overlap"].as<int>(3) << std::endl;

    std::cout << c.get<Config::Params::topic_refresh_rate, double>() << std::endl; 

    // std::cout << c.GET(subscribe_to_all) << std::endl;
    auto q = c.get<Config::Params::topics, std::vector<std::string>>();
    for(auto t : c.get<Config::Params::topics, std::vector<std::string>>()){
        std::cout << t << std::endl; 
    }

    // std::cout << config['topics'].as_array()[0] << std::endl;

    // std::string topic = "topics";
    // for(auto it=config[topic].as_array().begin(); it!=config[topic].as_array().end(); ++it){
    //     std::cout << it->as<std::string>() << std::endl;
    // }
}
