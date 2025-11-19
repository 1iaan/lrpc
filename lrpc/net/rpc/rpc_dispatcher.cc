#include "lrpc/common/error_code.h"
#include "lrpc/common/log.h"
#include "lrpc/net/rpc/rpc_dispatcher.h"
#include "lrpc/common/run_time.h"
#include "lrpc/net/coder/tinypb_protocol.h"
#include "lrpc/net/rpc/rpc_controller.h"
#include "lrpc/net/tcp/net_addr.h"
#include "lrpc/net/tcp/tcp_connection.h"
#include <cstddef>
#include <google/protobuf/descriptor.h>
#include <google/protobuf/message.h>
#include <memory>

namespace lrpc{

static RpcDispatcher* g_rpc_dispatcher= nullptr;

RpcDispatcher* RpcDispatcher::GetRpcDispatcher(){
    if(!g_rpc_dispatcher) g_rpc_dispatcher = new RpcDispatcher();
    return g_rpc_dispatcher;
}

void RpcDispatcher::dispatch(AbstractProtocol::s_ptr request, AbstractProtocol::s_ptr response, TcpConnection* connection){
    TinyPBProtocol::s_ptr req_protocol = std::dynamic_pointer_cast<TinyPBProtocol>(request);
    TinyPBProtocol::s_ptr rsp_protocol = std::dynamic_pointer_cast<TinyPBProtocol>(response);
    
    std::string method_full_name = req_protocol->method_name_;
    std::string service_name, method_name;

    rsp_protocol->msg_id_ = req_protocol->msg_id_;
    rsp_protocol->method_name_ = req_protocol->method_name_;

    if(parseServiceFullName(method_full_name, service_name, method_name) == false){
        setTinyPBError(rsp_protocol, ERROR_PARSE_SERVICE_NAME, "parse service name error");
        return;
    }
    auto it = service_map_.find(service_name);
    if(it == service_map_.end()){
        ERRORLOG("%s | service name[%s] not found", req_protocol->msg_id_.c_str(), service_name.c_str());
        setTinyPBError(rsp_protocol, ERROR_SERVICE_NOT_FOUND, "service not found");
        return;
    }

    service_s_ptr service = (*it).second;

    const google::protobuf::MethodDescriptor* method = service->GetDescriptor()->FindMethodByName(method_name);
    if(method == NULL){
        ERRORLOG("%s | method name[%s] not found", req_protocol->msg_id_.c_str(), method_name.c_str());
        setTinyPBError(rsp_protocol, ERROR_METHOD_NOT_FOUND, "method not found");
        return;
    }

    google::protobuf::Message* req_msg = service->GetRequestPrototype(method).New();

    // 反序列化
    if(!req_msg->ParseFromString(req_protocol->pb_data_)){
        ERRORLOG("%s | deserialize error", req_protocol->msg_id_.c_str());
        setTinyPBError(rsp_protocol, ERROR_FAILED_DESERIALIZE, "deserialize not found");
        if(req_msg != NULL){
            delete req_msg;
            req_msg = NULL;
        }
        return;
    }
    INFOLOG("[RpcDisPatcher] [%s] | get rpc request[%s]", req_protocol->msg_id_.c_str(), req_msg->ShortDebugString().c_str());

    google::protobuf::Message* rsp_msg = service->GetResponsePrototype(method).New();

    RpcController controller;
    controller.setLocalAddr(connection->getLocalAddr());
    controller.setPeerAddr(connection->getPeerAddr());
    controller.setMsgId(req_protocol->msg_id_);

    RunTime::GetRunTime()->msg_id_ = req_protocol->msg_id_;
    RunTime::GetRunTime()->method_name_ = req_protocol->method_name_;
    service->CallMethod(method, &controller, req_msg, rsp_msg, NULL);

    if(!rsp_msg->SerializeToString(&rsp_protocol->pb_data_)){
        ERRORLOG("%s | serialize error, origin message[%s]", req_protocol->msg_id_.c_str(), rsp_msg->ShortDebugString().c_str());
        setTinyPBError(rsp_protocol, ERROR_FAILED_SERIALIZE, "serialize not found");
        if(req_msg != NULL){
            delete req_msg;
            req_msg = NULL;
        }
        if(rsp_msg != NULL){
            delete rsp_msg;
            rsp_msg = NULL;
        }
        return;
    }

    rsp_protocol->err_code_ = 0;

    INFOLOG("[RpcDisPatcher] [%s] | dispatch success, request[%s], response[%s]",req_protocol->msg_id_.c_str(), req_msg->ShortDebugString().c_str(), rsp_msg->ShortDebugString().c_str());
    if(req_msg){
        delete req_msg;
        req_msg = NULL;
    }
    if(rsp_msg){
        delete rsp_msg;
        rsp_msg = NULL;
    }
}

void RpcDispatcher::registerService(service_s_ptr service){
    std::string service_name = service->GetDescriptor()->full_name();
    service_map_[service_name] = service;
}

bool RpcDispatcher::parseServiceFullName(const std::string& full_name, std::string &service_name, std::string &method_name){
    if(full_name.empty()){
        ERRORLOG("full_name empty"); 
        return false;
    }
    size_t i = full_name.find('.');
    if(i == full_name.npos){
        ERRORLOG("not find . in full_name[%s]", full_name.c_str());
        return false;
    }
    service_name = full_name.substr(0, i);
    method_name = full_name.substr(i+1, full_name.length() - i - 1);

    INFOLOG("[RpcDispatcher] parse service_name[%s], and method_name[%s] from full name [%s]",service_name.c_str(), method_name.c_str(), full_name.c_str());
    return true;
}

void RpcDispatcher::setTinyPBError(TinyPBProtocol::s_ptr msg, int32_t err_code, std::string err_info){
    msg->err_code_ = err_code;
    msg->err_info_ = err_info;
    msg->err_info_len_ = err_info.length();
}

} // namespace lrpc