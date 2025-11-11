#pragma once

#include "lrpc/net/tcp/net_addr.h"
#include "lrpc/net/tcp/tcp_client.h"
#include <google/protobuf/message.h>
#include <google/protobuf/service.h>
#include <google/protobuf/stubs/callback.h>
#include <memory>
namespace lrpc {

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

    void init(controller_s_ptr controller, message_s_ptr req, message_s_ptr res, closure_s_ptr done);

    google::protobuf::RpcController* getController(){ return controller_.get(); }
    google::protobuf::Message* getRequest(){ return request_.get(); }
    google::protobuf::Message* getResponse(){ return response_.get(); }
    google::protobuf::Closure* getClosure(){ return closure_.get(); }

    TcpClient* getTcpClient(){ return client_.get(); }

private:
    NetAddr::s_ptr local_addr_{nullptr};
    NetAddr::s_ptr peer_addr_{nullptr};

    controller_s_ptr controller_{nullptr};
    message_s_ptr request_{nullptr};
    message_s_ptr response_{nullptr};
    closure_s_ptr closure_{nullptr};

    bool is_init_{false};

    TcpClient::s_ptr client_{nullptr};
};

} // namespace lrpc