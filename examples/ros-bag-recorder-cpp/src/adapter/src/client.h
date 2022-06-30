#include <iostream>
#include <grpcpp/grpcpp.h>
#include <utility>

#include "protos/agent/v1/agent.grpc.pb.h"

using grpc::Channel;
using grpc::ClientContext;
using grpc::Status;

using v1::agent::Agent;

// For getting config data
using v1::agent::GetApplicationConfigurationRequest;
using v1::agent::GetApplicationConfigurationResponse;

// For receiving commands
using v1::agent::GetCommandRequestStreamRequest;
using v1::agent::GetCommandRequestStreamResponse;
using v1::agent::SendCommandResponseRequest;
using v1::agent::SendCommandResponseResponse;


#ifndef GET_CONFIG_H
#define GET_CONFIG_H

class FormantAgentClient
{
public:
   inline FormantAgentClient(std::shared_ptr<Channel> channel) : stub_(Agent::NewStub(channel)) {}

   inline FormantAgentClient() : FormantAgentClient(grpc::CreateChannel("unix:///var/lib/formant/agent.sock", grpc::InsecureChannelCredentials()))
   {
   }

   /**
    * @brief Return the configuration value for the key. Returned as a std::pair where the first element is
    *        true is the key was found, and returned as false if the key was not found in the config. 
    * 
    * @param param 
    * @return std::pair<bool, std::string> 
    */
   inline std::pair<bool, std::string> get_config_param(const std::string &key)
   {

      if (app_config_loaded) 
      {

         if (app_config.configuration_map().contains(key))
         {
            return {true, app_config.configuration_map().find(key)->second};
         }

         return {false, ""};
      }

      ClientContext context;
      GetApplicationConfigurationRequest request;
      GetApplicationConfigurationResponse response;

      stub_->GetApplicationConfiguration(&context, request, &response);

      app_config = response.configuration();
      app_config_loaded = true;

      return get_config_param(key);
   }




private:
   std::unique_ptr<Agent::Stub> stub_;

   v1::model::ApplicationConfiguration app_config;
   bool app_config_loaded = false;
};

#endif
