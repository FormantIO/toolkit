#include <iostream>
#include <grpcpp/grpcpp.h>
#include <utility>
#include <boost/thread/mutex.hpp>
#include <unordered_map>
#include <thread>
#include <functional>

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

// For getting blob data
using v1::agent::GetConfigBlobDataRequest; 
using v1::agent::GetConfigBlobDataResponse; 
using v1::model::BlobData;  

#ifndef GET_CONFIG_H
#define GET_CONFIG_H

class FormantAgentClient
{
public:
    using FormantAgentCallback = std::function<void(const GetCommandRequestStreamResponse &msg)>;

    inline FormantAgentClient(std::shared_ptr<Channel> channel) : stub_(Agent::NewStub(channel)) {}

    inline FormantAgentClient() : FormantAgentClient(grpc::CreateChannel("unix:///var/lib/formant/agent.sock", grpc::InsecureChannelCredentials()))
    {}

    inline ~FormantAgentClient()
    {
        if (command_stream_thread_started)
        {
            cancel_command_receiver_thread();
        }
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

        GetApplicationConfigurationRequest request;
        GetApplicationConfigurationResponse response;
        ClientContext context;

        stub_->GetApplicationConfiguration(&context, request, &response);

        app_config = response.configuration();
        app_config_loaded = true;

        return get_config_param(key);
    }

    /**
     * @brief Register a call-back with the Formant agent for the specified command's
     *
     * @param callback
     * @param command_filters
     */
    inline void register_command_callback(FormantAgentCallback callback, const std::vector<std::string> &command_filters, bool reset_stream = true)
    {
        for (size_t i = 0; i < command_filters.size(); ++i)
            register_command_callback(callback, command_filters[i], i == command_filters.size() - 1 && reset_stream);
    }

    /**
     * @brief Register a call-back with the Formant agent for the specified command
     *
     * @param callback
     * @param command_filters
     */
    inline void register_command_callback(FormantAgentCallback callback, const std::string &command_filter, bool reset_stream = true)
    {

        // Step 1: register the register the command filter if needed
        if (!command_filters.count(command_filter))
        {
            command_filters.insert(command_filter);
            command_stream_request.add_command_filter(command_filter);
        }
        else if (reset_stream)
        {
            reset_stream = false; // If we didn't add a new command filter, we dont need to reset stream
        }

        // Step 2: add the callback to the command filter
        {
            boost::mutex::scoped_lock lock(callbacks_map_mutex);
            callbacks[command_filter].push_back(callback);
        }

        // Step 3: if the stream has not started, then start the stream.
        if (!command_stream_thread_started)
        {
            start_command_receiver_thread();
            reset_stream = false;
        }

        // Step 4: reset the stream if needed
        if (reset_stream)
        {
            cancel_command_receiver_thread();
            start_command_receiver_thread();
        }
    }

    /**
     * @brief Get blob data from Formant
     * 
     * @return std::string 
     */
    inline std::string get_blob_data(){

        BlobData bd; 

        ClientContext context; 
        GetConfigBlobDataRequest request; 
        GetConfigBlobDataResponse response; 

        stub_->GetConfigBlobData(&context, request, &response); 

        return response.blob_data().data(); 
    }

private:
    /**
     * @brief The command receiver thread is a cancellable thread which waits for registered commands to
     *        be processed by the thread.
     *
     */
    inline void command_receiver_thread()
    {

        ClientContext stream_context;
        register_command_receiver_context(&stream_context);
        GetCommandRequestStreamRequest request = command_stream_request;
        auto stream = stub_->GetCommandRequestStream(&stream_context, request);
        GetCommandRequestStreamResponse message;

        command_stream_thread_started = true;

        while (stream->Read(&message))
        {
            auto command = message.request().command();
            auto payload = message.request().text();
            auto id = message.request().id();

            SendCommandResponseRequest finishRequest;
            SendCommandResponseResponse finishResponse;

            ClientContext response_context;
            finishRequest.mutable_response()->set_request_id(id);
            finishRequest.mutable_response()->set_success(true);
            stub_->SendCommandResponse(&response_context, finishRequest, &finishResponse);

            auto &callback_funcs = callbacks[command];

            // Pass the message to each callback for processing.
            for (auto f : callback_funcs)
                f(message);
        }
    }

    /**
     * @brief Stop the command receiver thread.
     *
     */
    inline void cancel_command_receiver_thread()
    {
        command_stream_context->TryCancel();
        command_stream_thread.join();
        command_stream_thread_started = false;
    }

    inline void start_command_receiver_thread()
    {
        command_stream_thread = std::thread(&FormantAgentClient::command_receiver_thread, this);
        while (!command_stream_thread_started)
            sleep(0.1);
    }

    inline void register_command_receiver_context(ClientContext *context)
    {
        command_stream_context = context;
    }

    // Agent stub for grpc
    std::unique_ptr<Agent::Stub> stub_;

    // variables for getting robot configuration from Formant
    v1::model::ApplicationConfiguration app_config;
    bool app_config_loaded = false;

    // the command stream variable which listens for specified commands.
    std::unique_ptr<grpc::ClientReader<v1::agent::GetCommandRequestStreamResponse>> command_stream;
    // std::unordered_map<std::string, std::vector<void (*)(const GetCommandRequestStreamResponse &msg)>> callbacks;
    std::unordered_map<std::string, std::vector<std::function<void(const GetCommandRequestStreamResponse &msg)>>> callbacks;

    boost::mutex callbacks_map_mutex;
    std::set<std::string> command_filters;
    GetCommandRequestStreamRequest command_stream_request; // request used to set-up the command stream with the agent.

    ClientContext *command_stream_context;

    std::thread command_stream_thread;
    bool command_stream_thread_started = false;
};

#endif