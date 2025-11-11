#include "lrpc/net/rpc/rpc_channel.h"
#include "lrpc/common/error_code.h"
#include "lrpc/common/log.h"
#include "lrpc/net/coder/abs_protocol.h"
#include "lrpc/net/coder/tinypb_protocol.h"
#include "lrpc/common/msg_id_util.h"
#include "lrpc/net/rpc/rpc_controller.h"
#include "lrpc/net/tcp/tcp_client.h"
#include "lrpc/net/timer_event.h"
#include <google/protobuf/message.h>
#include <cstddef>
#include <memory>
#include <string>

namespace lrpc {
RpcChannel::RpcChannel(NetAddr::s_ptr peer_addr): peer_addr_(peer_addr){
    client_ = std::make_shared<TcpClient>(peer_addr_);
}

RpcChannel::~RpcChannel(){

}

void RpcChannel::CallMethod(const google::protobuf::MethodDescriptor* method,
                    google::protobuf::RpcController* controller, const google::protobuf::Message* request,
                    google::protobuf::Message* response, google::protobuf::Closure* done){
    std::shared_ptr<TinyPBProtocol> req_protocol = std::make_shared<TinyPBProtocol>();
    
    RpcController* m_controller = dynamic_cast<RpcController*>(controller);
    if(m_controller == NULL){
        ERRORLOG("failed callmethod, RpcController convert error");
        return;
    }

    if(m_controller->getMsgId().empty()){
        req_protocol->msg_id_ = MsgIDUtil::GenMsgID();
        m_controller->setMsgId(req_protocol->msg_id_);
    }else{
        req_protocol->msg_id_ = m_controller->getMsgId();
    }

    req_protocol->method_name_ = method->full_name();
    INFOLOG("[%s] | call method name [%s]", req_protocol->msg_id_.c_str(), req_protocol->method_name_.c_str());

    if(!is_init_){
        std::string err_info = "RpcChannel not init";
        m_controller->setError(ERROR_CHANNEL_INIT, err_info);
        ERRORLOG("%s | %s", req_protocol->msg_id_.c_str(), err_info.c_str());
        return ;
    }

    
    if(!request->SerializeToString(&req_protocol->pb_data_)){
        std::string err_info = "failed to serialize";
        m_controller->setError(ERROR_FAILED_SERIALIZE, err_info);
        ERRORLOG("%s | %s, origin request [%s]", req_protocol->msg_id_.c_str(), err_info.c_str(), request->ShortDebugString().c_str());
        return ;
    }

    s_ptr channel = shared_from_this();

    timer_event_ = std::make_shared<TimerEvent>(m_controller->getTimeOut(), false, [m_controller, channel]() mutable->void{
        m_controller->StartCancel();
        m_controller->setError(ERROR_FAILED_SERIALIZE, "rpc call timeout " + std::to_string(m_controller->getTimeOut()));
    
        if(channel->getClosure()){
            channel->getClosure()->Run();
        }
        channel.reset();
        
    });

    client_->addTimerEvent(timer_event_);
    
    client_->connect([req_protocol, channel]()->void{

        RpcController* m_controller =  dynamic_cast<RpcController*>(channel->getController());

        if(channel->getTcpClient()->getConnectErrorCode() != 0) {
            ERRORLOG("%s | connect error, error code[%d], error info[%s], peer addr[%s]", 
                req_protocol->msg_id_.c_str(), channel->getTcpClient()->getConnectErrorCode(), 
                channel->getTcpClient()->getConnectErrorInfo().c_str(), channel->getTcpClient()->getPeerAddr()->toString().c_str());
            m_controller->setError(channel->getTcpClient()->getConnectErrorCode(), channel->getTcpClient()->getConnectErrorInfo());
            return;
        }

        channel->getTcpClient()->writeMessage(req_protocol, [req_protocol, channel](AbstractProtocol::s_ptr)->void{
            INFOLOG("%s | send request success, callmethod name[%s], origin request[%s], local_addr_[%s]", 
                req_protocol->msg_id_.c_str(), req_protocol->method_name_.c_str(), channel->getRequest()->ShortDebugString().c_str(),
                channel->getTcpClient()->getLocalAddr()->toString().c_str());
            
            channel->getTcpClient()->readMessage(req_protocol->msg_id_, [channel](AbstractProtocol::s_ptr msg) mutable ->void{
                std::shared_ptr<TinyPBProtocol> rsp_protocol = std::dynamic_pointer_cast<TinyPBProtocol>(msg);
                
                // 取消定时任务
                channel->getTimerEvent()->setCanceled(true);

                RpcController* m_controller =  dynamic_cast<RpcController*>(channel->getController());
                if(!channel->getResponse()->ParseFromString(rsp_protocol->pb_data_)){
                    ERRORLOG("serialize error");
                    m_controller->setError(ERROR_FAILED_DESERIALIZE, "deserialize error");
                    return;
                }

                INFOLOG("%s | success get rpc response, callmethod name[%s], origin request[%s]", 
                    rsp_protocol->msg_id_.c_str(), rsp_protocol->method_name_.c_str(), channel->getResponse()->ShortDebugString().c_str());
        
                if(rsp_protocol->err_code_ != 0){
                    ERRORLOG("%s | call rpc method[%s] failed, error code[%d], err info[%s]", 
                        rsp_protocol->err_code_, rsp_protocol->method_name_.c_str(), rsp_protocol->err_info_.c_str());
                    m_controller->setError(rsp_protocol->err_code_, rsp_protocol->err_info_);
                }

                if(!channel->getController()->IsCanceled() && channel->closure_){
                    channel->closure_->Run();
                }

                channel.reset();
            });
        });

    });
}

void RpcChannel::init(controller_s_ptr controller, message_s_ptr req, message_s_ptr res, closure_s_ptr done){
    if(is_init_){
        return;
    }
    controller_ = controller;
    request_ = req;
    response_ = res;
    closure_ = done;
    is_init_ = true;
}

} // namespace lrpc