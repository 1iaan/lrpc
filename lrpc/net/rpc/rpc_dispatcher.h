#pragma once

#include "lrpc/net/coder/abs_protocol.h"
#include "lrpc/net/coder/tinypb_protocol.h"
#include "lrpc/net/tcp/tcp_connection.h"
#include <map>
#include <string>
#include <memory>
#include <google/protobuf/service.h>
namespace lrpc{

class RpcDispatcher{
public:
    typedef std::shared_ptr<google::protobuf::Service> service_s_ptr;

    void dispatch(AbstractProtocol::s_ptr request, AbstractProtocol::s_ptr response, TcpConnection* connection);

    void registerService(service_s_ptr service);

    void setTinyPBError(TinyPBProtocol::s_ptr msg, int32_t err_code, std::string err_info);

public:
    static RpcDispatcher* GetRpcDispatcher();

private:
    bool parseServiceFullName(const std::string& full_name, std::string &service_name, std::string &method_name);

private:
    std::map<std::string, service_s_ptr> service_map_;
};

} // namespace lrpc