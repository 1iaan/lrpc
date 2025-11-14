#pragma once

#include "lrpc/common/mutex.h"
#include "lrpc/net/tcp/net_addr.h"
#include "lrpc/net/tcp/tcp_client.h"
#include "lrpc/net/timer.h"
#include "lrpc/net/timer_event.h"
#include <google/protobuf/message.h>
#include <google/protobuf/service.h>
#include <google/protobuf/stubs/callback.h>
#include <map>
#include <memory>
#include <string>
namespace lrpc {

#define NEWMESSAGE(type, var_name) \
    std::shared_ptr<type> var_name = std::make_shared<type>();   \

#define NEWRPCCONTROLLER(var_name) \
    std::shared_ptr<lrpc::RpcController> var_name = std::make_shared<lrpc::RpcController>();  \

#define NEWRPCCHANNEL(addr, var_name) \
    std::shared_ptr<lrpc::RpcChannel> var_name = std::make_shared<lrpc::RpcChannel>(std::make_shared<lrpc::IPNetAddr>(addr));  \

#define CALLRPC(stub_type, channel, method_name, controller, request, response, closure) \
    {                                                                                   \
        (channel)->addPendingCall(controller, request, response, closure);              \
        stub_type stub((channel).get());                                                \
        stub.method_name((controller).get(), (request).get(), (response).get(), (closure).get()); \
    }

class RpcChannel : public google::protobuf::RpcChannel, public std::enable_shared_from_this<RpcChannel>{
public:
    typedef std::shared_ptr<RpcChannel> s_ptr;
    typedef std::shared_ptr<google::protobuf::RpcController> controller_s_ptr;
    typedef std::shared_ptr<google::protobuf::Message>       message_s_ptr;
    typedef std::shared_ptr<google::protobuf::Closure>       closure_s_ptr;
public:
    RpcChannel(NetAddr::s_ptr peer_addr);

    ~RpcChannel();

    void CallMethod(const google::protobuf::MethodDescriptor* method,
                        google::protobuf::RpcController* controller, const google::protobuf::Message* request,
                        google::protobuf::Message* response, google::protobuf::Closure* done);

    void addPendingCall(controller_s_ptr controller, message_s_ptr req, message_s_ptr res, closure_s_ptr done);
    
    void delPendingCall(std::string msg_id);

    google::protobuf::RpcController* getController(std::string msg_id);
    google::protobuf::Message* getRequest(std::string msg_id);
    google::protobuf::Message* getResponse(std::string msg_id);
    google::protobuf::Closure* getClosure(std::string msg_id);
    TcpClient* getTcpClient(){ return client_.get(); }
    
    TimerEvent* getTimerEvent(std::string msg_id);

    void addTimerEvent(std::string msg_id, TimerEvent::s_ptr event);

    void stop();
public:
    struct PendingCall {
        controller_s_ptr controller;
        message_s_ptr request;
        message_s_ptr response;
        closure_s_ptr closure;
        TimerEvent::s_ptr timer;
    };

private:
    NetAddr::s_ptr local_addr_{nullptr};
    NetAddr::s_ptr peer_addr_{nullptr};

    // controller_s_ptr controller_{nullptr};
    // message_s_ptr request_{nullptr};
    // message_s_ptr response_{nullptr};
    // closure_s_ptr closure_{nullptr};
    // TimerEvent::s_ptr timer_event_{nullptr};

    bool is_init_{false};

    TcpClient::s_ptr client_{nullptr};

    std::map<std::string, PendingCall> pending_calls_;
    Mutex pending_mutex_;

    sem_t conn_semaphore_;
};

} // namespace lrpc